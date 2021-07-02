// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_custom_controllers/AIC_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_custom_controllers {

void AICController::setGoalMuDCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for( int i = 0; i < 7; i++ ) {
      desired_q[i] = msg->data[i];
    }
  }

bool AICController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;

  // Subscriber
  goal_mu_dSub = node_handle.subscribe("/GoalPositions", 5, &AICController::setGoalMuDCallback, this);

  ROS_WARN(
      "AICController: Ready to start the magic?!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("AICController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "AICController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("AICController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AICController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("AICController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AICController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("AICController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("AICController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void AICController::starting(const ros::Time& /*time*/) {
  // Initialise the controller's parameters
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Set  a default goal position
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
  k_a = 200;

  // Integration step
  h = 0.001;

  // Precision matrices (first set them to zero then populate the diagonal)
  SigmaP_yq0 = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_yq1 = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_mu = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_muprime = Eigen::Matrix<double, 7, 7>::Zero();
  //////////////////////////////////////////////

  // Initialization
  for( int i = 0; i < 7; i = i + 1 ) {
    SigmaP_yq0(i,i) = 1/var_q;
    SigmaP_yq1(i,i) = 1/var_qdot;
    SigmaP_mu(i,i) = 1/var_mu;
    SigmaP_muprime(i,i) = 1/var_muprime;
    // Internal belief starting from initial pose
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
  }
  // Initial belief
  mu = jointPos;
  mu_p = jointVel;
  mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // Initial control actions
  u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

 }

void AICController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Save the robot state for algebric manipulation
  for( int i = 0; i < 7; i = i + 1 ) {
    // Set current sensory input
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
  }

  // Set the currently published values
  mu_d = desired_q;

  /////////// Active inference ///////////
  // Belief update
  mu_dot = mu_p - k_mu*(-SigmaP_yq0*(jointPos-mu)+SigmaP_mu*(mu_p+mu-mu_d));
  mu_dot_p = mu_pp - k_mu*(-SigmaP_yq1*(jointVel-mu_p)+SigmaP_mu*(mu_p+mu-mu_d)+SigmaP_muprime*(mu_pp+mu_p));
  mu_dot_pp = - k_mu*(SigmaP_muprime*(mu_pp+mu_p));

  mu = mu + h*mu_dot;             // Belief about the position
  mu_p = mu_p + h*mu_dot_p;       // Belief about motion of mu
  mu_pp = mu_pp + h*mu_dot_pp;    // Belief about motion of mu'

  // Compute control actions
  // This is  quick fix to tune the last joint separately, you should not do like that but define k_a as a vector

  uSix_dummy = u(6)-h*0.4*k_a*(SigmaP_yq1(6,6)*(jointVel(6)-mu_p(6))+SigmaP_yq0(6,6)*(jointPos(6)-mu(6)));
  uFive_dummy = u(5)-h*0.5*k_a*(SigmaP_yq1(5,5)*(jointVel(5)-mu_p(5))+SigmaP_yq0(5,5)*(jointPos(5)-mu(5)));
  u = u-h*k_a*(SigmaP_yq1*(jointVel-mu_p)+SigmaP_yq0*(jointPos-mu));
  u(6) = uSix_dummy;
  u(5) = uFive_dummy;

  //This is the original contol actions update
  //u = u-h*k_a*(SigmaP_yq1*(jointVel-mu_p)+SigmaP_yq0*(jointPos-mu));

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(u(i));
  }
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_custom_controllers::AICController,
                       controller_interface::ControllerBase)
