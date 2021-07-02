// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
 * File:   uAICcontroller.h
 * Author: Corrado Pezzato, TU Delft, DCSC
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
#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Core>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"


namespace franka_custom_controllers {

class uAICController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  // Mandatory from dicumentation
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  // Subscriber
  void setGoalMuDCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
 private:

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  Eigen::Matrix<double, 7, 1> desired_q;
  // Define variables for the controller
  // Variances for internal belief and sensory input, learning rates, integration step
  double var_q, var_qdot, var_mu, var_muprime, k_mu, h;
  // Controller parameters, PID like control law. Diagonal matrices
  Eigen::Matrix<double, 7, 7> K_p, K_i, K_d;

  // Precision matrices, diagonal matrices with the inverce of the variance
  Eigen::Matrix<double, 7, 7> SigmaP_yq0, SigmaP_yq1, SigmaP_mu, SigmaP_muprime;
  // Belief about the states and their derivatives mu, mu', mu'', joint states, desired value, control
  Eigen::Matrix<double, 7, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel, mu_d, u, mu_past, mu_p_past, I_gain, mu_p_d;
  // Via-points
  Eigen::Matrix<double, 7, 1>  mu_d_prePush;
  // Parameters for control law, to populate the gain matrices
  double  k_p, k_d, k_i;
  ros::Subscriber goal_mu_dSub;
};

}  // namespace franka_custom_controllers
