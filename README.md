# Active_inference
This repository contains the C++ implementation of active inference for robot control.

### How to cite this work
This package was developed to support the theoretical results of this work:

- Pezzato C., Ferrari, R., Henrandez C., *A Novel Adaptive Controller for Robot Manipulators Based on Active Inference*, IEEE Robotics and Automation Letters, 2020. 

If you found this controller useful, please consider citing the paper above. 

## C++ scripts to control the Franka Emika Panda
This repository contains the implementation of active infernce to control the real Franka Emika Panda 7-DOF robot arm. 
The control is going to be in the joint space using torque commands and position/velocity sensors.

### Bare-bones C++ scripts
"AICpanda.cpp" makes the robot move to a desired goal. 
"AICpandapickplace" makes the robot perform a cycle which will be completed with the pick and place of objects later on.

### ROS implementation
The repository also contains a ROS package to use the ROS infrasctucture to control the Panda. The package should be installed within franka_ros (https://github.com/frankaemika/franka_ros)

### Video
A video clip showing a comparison between a model reference adaptive controller (MRAC) and the novel active inference controller (AIC) is available at https://youtu.be/Vsb0MzOp_TY
