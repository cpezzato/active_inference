/*
 * File:   AICpanda.cpp
 * Author: Corrado Pezzato, CoR, TU Deft
 *
 * Created on September 4th, 2019
 *
 * This node allows to control the 7DOF Franka Emika Panda robot arm through
 * Active Inference
 *
 * The control is in joint space, sending torque commands and using position and
 * velocity feedback. The robot performs a cycle of movements to 3 set-points
 *
 * ---> Usage:: ./<folder_code>/AICpandapickplace.cpp <fci-ip>
 *
 */

#include <iostream>
#include <array>
#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "examples_common.h"

int main(int argc, char** argv) {
 // Check if the user provided the hostname or IP
 if (argc != 2) {
   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
   return -1;
 }
 // If hostname or IP provided, try to connect to the robot
 try {

   // Define an object for the Panda to handle the control signals and read
   // the sensors. Using the method robot.control() you read the
   // joint values at 1kHz
   franka::Robot robot(argv[1]);
   setDefaultBehavior(robot);

   // Set collision behavior
   robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

   // Define controller
   /////////// Controller parameters /////////////
   // Auxiliary variables
   double time = 0.0;
   double maxTime = 30;
   // Variances for internal belief and sensory input, learning rates, integration step
   double var_q, var_qdot, var_mu, var_muprime, k_mu, k_a, h;
   // Precision matrices, diagonal matrices with the inverce of the variance
   Eigen::Matrix<double, 7, 7> SigmaP_yq0, SigmaP_yq1, SigmaP_mu, SigmaP_muprime;
   // Belief about the states and their derivatives mu, mu', mu'', joint states, prior, control
   Eigen::Matrix<double, 7, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel, mu_d, u;
   // Via-points
   Eigen::Matrix<double, 7, 1>  mu_dRight, mu_dGrab, mu_dCenter, mu_dRelease;

   // Set desired goal
   mu_dRight << 0.15086,0.235858,-0.458875,-2.44786,0.0599695,2.6241,0.450365;
   mu_dGrab << 0.158216,0.391831,-0.450989,-2.40055,0.169372,2.64562,0.338242;
   mu_dCenter << 0.0345169,-0.167844,-0.000332711,-1.74526,0.00905814,1.56589,0.824567;
   mu_dRelease << 0.742635,0.336485,-0.394359,-2.43545,0.264676,2.74637,0.90164;
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

   // Read robot initial state to set the initial belief
   franka::RobotState initial_state = robot.readOnce();
   // Initialization
   for( int i = 0; i < 7; i = i + 1 ) {
     SigmaP_yq0(i,i) = 1/var_q;
     SigmaP_yq1(i,i) = 1/var_qdot;
     SigmaP_mu(i,i) = 1/var_mu;
     SigmaP_muprime(i,i) = 1/var_muprime;
     // Internal belief starting from initial pose
     jointPos(i) = initial_state.q[i];
     jointVel(i) = initial_state.dq[i];
   }
   // Initial belief
   mu = jointPos;
   mu_p = jointVel;
   mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
   // Initial control actions
   u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

   // Controller callback for RT loop
   auto my_AIC_controller_callback = [&](const franka::RobotState& robot_state,
                                     franka::Duration period) -> franka::Torques {
     time += period.toSec();
     // Save the robot state for algebric manipulation
     for( int i = 0; i < 7; i = i + 1 ) {
       // Set current sensory input
       jointPos(i) = robot_state.q[i];
       jointVel(i) = robot_state.dq[i];
     }

     // Set here the via-points with a simple time-based logic
     if (time == 0){
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
     if (time >= 24.0){
       mu_d = mu_dCenter;
     }
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

    // Build control signal and send it to the robot
    franka::Torques tau_cmd = {{u(0), u(1), u(2), u(3), u(4), u(5), u(6)}};
    // Set torques to zero for testing
    //franka::Torques tau_cmd = {{0, 0, 0, 0, 0, 0, 0}};

    if (time >= maxTime) {
      std::cout << std::endl << "Finished motion, shutting down demo" << std::endl;
      return franka::MotionFinished(tau_cmd);
    }
    return tau_cmd;
   };
   std::cout << "WARNING: Make sure sure that the robot has enough space to move. Keep in mind that "
                "collision thresholds are set to high values."
             << std::endl
             << "Press Enter to start the demo..." << std::endl;
   std::cin.ignore();
   // start real-time control loop
   robot.control(my_AIC_controller_callback);
   // Catch the exemptions if any connection errors occurred
 } catch (franka::Exception const& e) {
  std::cout << e.what() << std::endl;
     return -1;
   }
  return 0;
}
