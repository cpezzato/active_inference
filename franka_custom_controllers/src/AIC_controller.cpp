// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
* Modified version for active inference
*
* Author: Corrado Pezzato
* Date 09-09-2019
*
* This script implements an active inference controller for the Panda Franka
* Emika through the ROS framework.
*
*/

#include <franka_custom_controllers/AIC_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_custom_controllers {

bool AICController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;

  // Publishers
  aPub1 = node_handle.advertise<std_msgs::Float64>("AICactions1", 10);
  aPub2 = node_handle.advertise<std_msgs::Float64>("AICactions2", 10);
  aPub3 = node_handle.advertise<std_msgs::Float64>("AICactions3", 10);
  aPub4 = node_handle.advertise<std_msgs::Float64>("AICactions4", 10);
  aPub5 = node_handle.advertise<std_msgs::Float64>("AICactions5", 10);
  aPub6 = node_handle.advertise<std_msgs::Float64>("AICactions6", 10);
  aPub7 = node_handle.advertise<std_msgs::Float64>("AICactions7", 10);

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

  // Center
  mu_d << 0.0345169,-0.167844,-0.000332711,-1.74526,0.00905814,1.56589,0.824567;

  // Right
  //mu_d << 0.15086,0.235858,-0.458875,-2.44786,0.0599695,2.6241,0.450365;

  //////////// Controller tuning //////////////
  // Variances associated with the beliefs and the sensory inputs
  var_mu = 5.0;
  var_muprime = 10.0;
  // For no gravity use these
  //var_mu = 15.0;
  //var_muprime = 30.0;
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

  // Set initial time
  time = 0.0;

  // Set via-points
  mu_dRight << 0.15,0.23,-0.45,-2.40,0.0,2.6,0.45;
  mu_dGrab << 0.15,0.4,-0.45,-2.4,0.17,2.65,0.35;
  mu_dCenter << 0.0,-0.16,-0.00,-1.75,0.00,1.5,0.80;
  mu_dRelease << 0.75,0.35,-0.4,-2.4,0.25,2.75,0.90;
}

void AICController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  time += period.toSec();

  // Save the robot state for algebric manipulation
  for( int i = 0; i < 7; i = i + 1 ) {
    // Set current sensory input
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
  }

  // Set here the via-points
  if (time >= 0.0 && time < 6.0){
    mu_d = mu_dCenter;
  }
  if (time >= 6.0 && time < 12.0){
    mu_d = mu_dRight;
  }
  if (time >= 12.0 && time < 18.0){
    mu_d = mu_dCenter;
  }
  if (time >= 18.0 && time < 24.0){
    mu_d = mu_dRelease;
  }
  //if (time >= 24.0){
    //mu_d = mu_dCenter;
  //}

  /////////// Active inference ///////////
  // Belief update
  mu_dot = mu_p - k_mu*(-SigmaP_yq0*(jointPos-mu)+SigmaP_mu*(mu_p+mu-mu_d));
  mu_dot_p = mu_pp - k_mu*(-SigmaP_yq1*(jointVel-mu_p)+SigmaP_mu*(mu_p+mu-mu_d)+SigmaP_muprime*(mu_pp+mu_p));
  mu_dot_pp = - k_mu*(SigmaP_muprime*(mu_pp+mu_p));

  mu = mu + h*mu_dot;             // Belief about the position
  mu_p = mu_p + h*mu_dot_p;       // Belief about motion of mu
  mu_pp = mu_pp + h*mu_dot_pp;    // Belief about motion of mu'

  // Compute control actions
  u = u-h*k_a*(SigmaP_yq1*(jointVel-mu_p)+SigmaP_yq0*(jointPos-mu));

  // Publish control actions
  //for (int i=0;i<7;i++){
     a1.data = u(0);
     a2.data = u(1);
     a3.data = u(2);
     a4.data = u(3);
     a5.data = u(4);
     a7.data = u(5);
     a7.data = u(6);
  //}
  aPub1.publish(a1);
  aPub2.publish(a2);
  aPub3.publish(a3);
  aPub4.publish(a4);
  aPub5.publish(a5);
  aPub6.publish(a6);
  aPub7.publish(a7);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(u(i));
  }
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_custom_controllers::AICController,
                       controller_interface::ControllerBase)
