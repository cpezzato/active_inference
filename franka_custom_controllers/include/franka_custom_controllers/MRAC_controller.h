// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
 * File:   MRACcontroller.h
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on September 5th, 2019
 *
 * Header file for the model reference class. Definition of the variables for
 * the MRAC for the real Franka Emika Panda robot arm
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>

namespace franka_custom_controllers {

class MRACController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  // Mandatory from dicumentation
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Define variables for the controller
  // Variables for sensory information and control input
  Eigen::Matrix<double, 7, 1> jointPos, jointVel, u;
  // Desired robot's states and error, column vector of 7 elements
  Eigen::Matrix<double, 7, 1> qr, dqr, qe, x_qe, qe_integral;
  // States to perform integrals of the error signal simulating a first order system as integrator
  Eigen::Matrix<double, 7, 7> X_qr, X_dqr, X_q, X_dq, qe_q_integral, qe_qr_integral, qe_dq_integral, qe_dqr_integral;
  // Support variable to control the flow of the script
  // MRAC Variables
  // Natural frequencies and damping ratio
  Eigen::Matrix<double, 7, 1> omega, zeta;
  // Adaptive gains and initial guesses
  Eigen::Matrix<double, 7, 7> K0, K1, Q0, Q1, K0_hat, K1_hat, Q0_hat, Q1_hat;
  // Controller parameters
  Eigen::Matrix<double, 7, 7> ALPHA1, ALPHA2, ALPHA3, E01, E02, E03, E11, E12, E13, F01, F02, F03, F11, F12, F13;
  // Support variables for adaptation law
  Eigen::Matrix<double, 7, 1> f, l1, l2, p2, p3, alpha1, alpha2, alpha3, e01, e02, e03, e11, e12, e13, f01, f02, f03, f11, f12, f13;
  Eigen::Matrix<double, 7, 7> P2, P3;
  // Integration step
  double h;
  // Via-points
  Eigen::Matrix<double, 7, 1>  qr_dRight, qr_dGrab, qr_dCenter, qr_dRelease;
  // Auxiliary variables
  double time = 0.0;
};

}  // namespace franka_custom_controllers
