// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
 * File:   AICcontroller.h
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on September 5th, 2019
 *
 * Class to perform active inference control of the 7DOF Franka Emika Panda robot.
 *
 * This class takes care of everything, it perfoms free-energy minimization
 * using gradient descent updating the beliefs about the rosbot's states
 * (i.e. joint values) and computing the control actions. The control
 * is in joint space.
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
#include "std_msgs/Float64.h"


namespace franka_custom_controllers {

class AICController : public controller_interface::MultiInterfaceController<
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
  // Variances for internal belief and sensory input, learning rates, integration step
  double var_q, var_qdot, var_mu, var_muprime, k_mu, k_a, h;
  // Auxiliari variables
  double time;
  // Precision matrices, diagonal matrices with the inverce of the variance
  Eigen::Matrix<double, 7, 7> SigmaP_yq0, SigmaP_yq1, SigmaP_mu, SigmaP_muprime;
  // Belief about the states and their derivatives mu, mu', mu'', joint states, desired value, control
  Eigen::Matrix<double, 7, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel, mu_d, u;
  // Via-points
  Eigen::Matrix<double, 7, 1>  mu_dRight, mu_dGrab, mu_dCenter, mu_dRelease;
  // Definition of variables in order to publish the control actions
  std_msgs::Float64 a1, a2, a3, a4, a5, a6, a7;
  // Publishers for beliefs
  ros::Publisher aPub1, aPub2, aPub3, aPub4, aPub5, aPub6, aPub7;
};

}  // namespace franka_custom_controllers
