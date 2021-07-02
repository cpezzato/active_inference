// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_custom_controllers/uAIC_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_custom_controllers {

void uAICController::setGoalMuDCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for( int i = 0; i < 7; i++ ) {
      desired_q[i] = msg->data[i];
      mu_p_d(i) = 0.0;
    }
  }

bool uAICController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;

  goal_mu_dSub = node_handle.subscribe("/franka_arm/AIC_controller/armGoalMultiArray", 5, &uAICController::setGoalMuDCallback, this);

  ROS_WARN(
      "uAICController: Ready to start the magic?!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("uAICController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "uAICController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("uAICController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "uAICController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("uAICController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "uAICController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("uAICController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("uAICController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void uAICController::starting(const ros::Time& /*time*/) {
  // Initialise the controller's parameters
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Set a default goal position
  mu_d << -0.017, -0.547, -0.04, -2.244, -0.013, 1.694, 0.709;
  desired_q = mu_d;

  //////////// Controller tuning //////////////
  // Variances associated with the beliefs and the sensory inputs
  var_mu = 5.0;
  var_muprime = 10.0;
  var_q = 1;
  var_qdot = 1;

  // Learning rates for the gradient descent
  k_mu = 11.67;

  // Integration step
  h = 0.001;

  // Precision matrices (first set them to zero then populate the diagonal)
  SigmaP_yq0 = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_yq1 = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_mu = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_muprime = Eigen::Matrix<double, 7, 7>::Zero();
  K_p = Eigen::Matrix<double, 7, 7>::Zero();
  K_d = Eigen::Matrix<double, 7, 7>::Zero();
  K_i = Eigen::Matrix<double, 7, 7>::Zero();

  //////////////////////////////////////////////
  // Initialization

  // Controller values, diagonal elements of the gain matrices for the PID like control law
  k_p = 25;
  k_d = 10;
  k_i = 0;
  I_gain <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  for( int i = 0; i < 7; i = i + 1 ) {
    SigmaP_yq0(i,i) = 1/var_q;
    SigmaP_yq1(i,i) = 1/var_qdot;
    SigmaP_mu(i,i) = 1/var_mu;
    SigmaP_muprime(i,i) = 1/var_muprime;
    // Internal belief starting from initial pose
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
    // Initialize prior over velocity
    mu_p_d(i) = 0.0;
    K_p(i,i) = k_p;
    K_d(i,i) = k_d;
    K_i(i,i) = k_i;
  }
  // Initial belief
  mu = jointPos;
  mu_p = jointVel;
  mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  mu_past = mu;
  mu_p_past = mu_p;

  // Initial control actions
  u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  mu_d_prePush << 1.51, -0.023, 0.049, -2.321, 0.034, 2.233, 0.807;
 }

void uAICController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Save the robot state for algebric manipulation
  for( int i = 0; i < 7; i = i + 1 ) {
    // Set current sensory input
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
  }

  // Set the currently published values
  mu_d = desired_q;
  //mu_d = mu_d_prePush;

  /////////// Active inference ///////////
  // Belief update
  // Unbiased AIC
  mu_dot = - k_mu*(-SigmaP_yq0*(jointPos-mu) + SigmaP_mu*(mu - (mu_past + h*mu_p_past)));
  mu_dot_p = - k_mu*(-SigmaP_yq1*(jointVel-mu_p) + SigmaP_muprime*(mu_p-mu_p_past));

  // Save current value of the belief to use in the next iteration as previous value
  mu_past = mu;
  mu_p_past = mu_p;

  mu = mu + h*mu_dot;             // Belief about the position
  mu_p = mu_p + h*mu_dot_p;       // Belief about motion of mu

  // Set curret values for next ieration
  I_gain = I_gain + mu_d-mu;

  // Compute control actions
  // Unbiased AIC
  u = K_p*(mu_d-mu) + K_d*(mu_p_d-mu_p) + K_i*(I_gain);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(u(i));
  }
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_custom_controllers::uAICController,
                       controller_interface::ControllerBase)
