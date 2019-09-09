// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
* Modified version for active inference
*
* Author: Corrado Pezzato
* Date 09-09-2019
*
* This script implements a model reference adaptive controller for the Panda Franka
* Emika through the ROS framework.
*
*/

#include <franka_custom_controllers/MRAC_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_custom_controllers {

bool MRACController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "MRACController: Ready to start.");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("MRACController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "MRACController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("MRACController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MRACController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("MRACController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MRACController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("MRACController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("MRACController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void MRACController::starting(const ros::Time& /*time*/) {
  // Initialise the controller's parameters
  franka::RobotState robot_state = state_handle_->getRobotState();
  // Center
  // Set desired goal
  qr_dRight << 0.15,0.23,-0.45,-2.40,0.0,2.6,0.45;
  qr_dGrab << 0.15,0.4,-0.45,-2.4,0.17,2.65,0.35;
  qr_dCenter << 0.0,-0.16,-0.00,-1.75,0.00,1.5,0.80;
  qr_dRelease << 0.75,0.35,-0.4,-2.4,0.25,2.75,0.90;

  // Initialize controller
  // Set goal for velocity (always zero), the goal for the position is set by the main node MRAC_controller_node
  dqr << 0, 0, 0, 0, 0, 0, 0;
  // Set initial conditions for the states for the integrals
  x_qe << 0, 0, 0, 0, 0, 0, 0;
  X_qr << Eigen::Matrix<double, 7, 7>::Zero();
  X_dqr << Eigen::Matrix<double, 7, 7>::Zero();
  X_q << Eigen::Matrix<double, 7, 7>::Zero();
  X_dq << Eigen::Matrix<double, 7, 7>::Zero();

  // Integration step
  h = 0.001;

  // Initialize control actions
  u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  //////////// Controller tuning //////////////
  // MRAC Variables
  omega << 2, 2, 2, 2, 2, 2, 2;
  // Damping
  zeta << 1, 1, 1, 1, 1, 1, 1;

  // Controller parameters
  alpha1 << 2, 10, 2, 10, 10, 10, 2;
  alpha2 << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
  alpha3 << 0, 0, 0, 0, 0, 0, 0;
  e01 << 1, 2, 1, 2, 1, 2, 1;
  e02 << 0.1, 0.2, 0.1, 0.2, 0.1, 0.2, 0.1;
  e03 << 0, 0, 0, 0, 0, 0, 0;
  e11 << 1, 2, 1, 2, 1, 2, 1;
  e12 << 0.1, 0.2, 0.1, 0.2, 0.1, 0.2, 0.1;
  e13 << 0, 0, 0, 0, 0, 0, 0;
  f01 << 1, 1, 1, 1, 5, 5, 5;
  f02 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  f03 << 0, 0, 0, 0, 0, 0, 0;
  // Not used for constant ref
  f11 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  f12 << 0.1, 0.1, 0.1, 0.1, 1, 1, 1;
  f13 << 0, 0, 0, 0, 0, 0, 0;
  ////////////////////////////
  l1 << 1, 1, 1, 1, 1, 1, 1;
  l2 << 1, 1, 1, 1, 1, 1, 1;

  // From here on a normal user should not modify
  p2 << l1[0]/(2*omega[0]*omega[0]), l1[1]/(2*omega[1]*omega[1]), l1[2]/(2*omega[2]*omega[2]),
        l1[3]/(2*omega[3]*omega[3]), l1[4]/(2*omega[4]*omega[4]), l1[5]/(2*omega[5]*omega[5]), l1[6]/(2*omega[6]*omega[6]);
  p3 << l2[0]/(4*zeta[0]*omega[0])+l1[0]/(4*zeta[0]*pow(omega[0],3)), l2[1]/(4*zeta[1]*omega[1])+l1[1]/(4*zeta[1]*pow(omega[1],3)),
        l2[2]/(4*zeta[2]*omega[2])+l1[2]/(4*zeta[2]*pow(omega[2],3)), l2[3]/(4*zeta[3]*omega[3])+l1[3]/(4*zeta[3]*pow(omega[3],3)),
        l2[4]/(4*zeta[4]*omega[4])+l1[4]/(4*zeta[4]*pow(omega[4],3)), l2[5]/(4*zeta[5]*omega[5])+l1[5]/(4*zeta[5]*pow(omega[5],3)),
        l2[6]/(4*zeta[6]*omega[6])+l1[6]/(4*zeta[6]*pow(omega[6],3));

  // Initial adaptive gains and constant matrices
  K0_hat = Eigen::Matrix<double, 7, 7>::Zero();
  K1_hat = Eigen::Matrix<double, 7, 7>::Zero();
  Q0_hat = Eigen::Matrix<double, 7, 7>::Zero();
  Q1_hat = Eigen::Matrix<double, 7, 7>::Zero();

  // Controller parameters
  ALPHA1 = Eigen::Matrix<double, 7, 7>::Zero();
  ALPHA2 = Eigen::Matrix<double, 7, 7>::Zero();
  ALPHA3 = Eigen::Matrix<double, 7, 7>::Zero();
  E01 = Eigen::Matrix<double, 7, 7>::Zero();
  E02 = Eigen::Matrix<double, 7, 7>::Zero();
  E03 = Eigen::Matrix<double, 7, 7>::Zero();
  E11 = Eigen::Matrix<double, 7, 7>::Zero();
  E12 = Eigen::Matrix<double, 7, 7>::Zero();
  E13 = Eigen::Matrix<double, 7, 7>::Zero();
  F01 = Eigen::Matrix<double, 7, 7>::Zero();
  F02 = Eigen::Matrix<double, 7, 7>::Zero();
  F03 = Eigen::Matrix<double, 7, 7>::Zero();
  F11 = Eigen::Matrix<double, 7, 7>::Zero();
  F12 = Eigen::Matrix<double, 7, 7>::Zero();
  F13 = Eigen::Matrix<double, 7, 7>::Zero();
  P2 = Eigen::Matrix<double, 7, 7>::Zero();
  P3 = Eigen::Matrix<double, 7, 7>::Zero();

  for( int i = 0; i < P2.rows(); i = i + 1 ) {
   ALPHA1(i,i) = alpha1(i);
   ALPHA2(i,i) = alpha2(i);
   ALPHA3(i,i) = alpha3(i);
   E01(i,i) = e01(i);
   E02(i,i) = e02(i);
   E03(i,i) = e03(i);
   E11(i,i) = e11(i);
   E12(i,i) = e12(i);
   E13(i,i) = e13(i);
   F01(i,i) = f01(i);
   F02(i,i) = f02(i);
   F03(i,i) = f03(i);
   F11(i,i) = f11(i);
   F12(i,i) = f12(i);
   F13(i,i) = f13(i);
   P2(i,i) = p2(i);
   P3(i,i) = p3(i);
  }
  // Set initial time
  time = 0.0;
}

void MRACController::update(const ros::Time& /*time*/, const ros::Duration& period) {
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
    qr = qr_dCenter;
  }
  if (time >= 6.0 && time < 12.0){
    qr = qr_dRight;
  }
  if (time >= 12.0 && time < 18.0){
    qr = qr_dCenter;
  }
  if (time >= 18.0 && time < 24.0){
    qr = qr_dRelease;
  }
  //if (time >= 24.0){
    //qr_d = qr_dCenter;
  //}

  /////////// MRAC ///////////
  // Definition of the modified joint angle error
  qe = P2*(qr-jointPos)+P3*(dqr-jointVel);

  // Feed-forward term f
  // Integral of qe using first order system as integrator
  qe_integral = x_qe;
  x_qe = x_qe + h*qe;
  f = ALPHA1*qe + ALPHA2*qe_integral;

  // Adaptive gain K0
  qe_q_integral = X_q;
  X_q = X_q + qe*jointPos.transpose();

  K0 = K0_hat + E01*(qe*jointPos.transpose()) + E02*qe_q_integral;

  // Adaptive gain K1
  qe_dq_integral = X_q;
  X_dq = X_dq + qe*jointVel.transpose();
  K1 = K1_hat + E11*(qe*jointVel.transpose()) + E12*qe_dq_integral;

  // Adaptive gain Q0
  qe_qr_integral = X_qr;
  X_qr = X_qr + qe*qr.transpose();
  Q0 = Q0_hat + F01*(qe*qr.transpose()) + F02*qe_qr_integral;

  // Adaptive gain K1
  qe_dqr_integral = X_dqr;
  X_dqr = X_dqr + qe*dqr.transpose();
  Q1 = Q1_hat + F11*(qe*dqr.transpose()) + F12*qe_dqr_integral;
  // Torque to the robot arm
  u = (K0*jointPos + K1*jointVel + Q0*qr + Q1*dqr + f);

  // Send commands
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(u(i));
  }
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_custom_controllers::MRACController,
                       controller_interface::ControllerBase)
