# Active_inference
This repository contains the C++ implementation of active inference for robot control.

## C++ scripts to control the Franka Emika Panda
This repository contains the implementation of active infernce to control the real Franka Emika Panda 7-DOF robot arm. 
The control is going to be in the joint space using torque commands and position/velocity sensors.

### Bare-bones C++ scripts
"AICpanda.cpp" makes the robot move to a desired goal. 
"AICpandapickplace" makes the robot perform a cycle which will be completed with the pick and place of objects later on.

#### ROS implementation
The repository also contains a ROS package to use the ROS infrasctucture to control the Panda. The package should be installed within franka_ros (https://github.com/frankaemika/franka_ros)
