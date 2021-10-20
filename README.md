# Active_inference
This repository contains the C++ implementation of active inference for robot control. For simulation purposes, this repo can be handy instead https://github.com/cpezzato/panda_simulation

### How to cite this work
This package was developed to support the theoretical results of this work:

- Pezzato C., Ferrari, R., Henrandez C., *A Novel Adaptive Controller for Robot Manipulators Based on Active Inference*, IEEE Robotics and Automation Letters, 2020. 

If you found this controller useful, please consider citing the paper above. 

## C++ scripts to control the Franka Emika Panda
This repository contains the implementation of active infernce to control the real Franka Emika Panda 7-DOF robot arm. 
The control is going to be in joint space using torque commands and position/velocity sensors. This can be done either using *libfranca* (https://frankaemika.github.io/docs/installation_linux.html) using the scripts *AICpanda.ccp* and *AICpickandplace.cpp*. Or, through ros infrastructure using the package *franka_custom_controllers*, to be installed within the official *franka_ros* https://frankaemika.github.io/docs/franka_ros.html. 

### Bare-bones C++ scripts for libfranka
"AICpanda.cpp" makes the robot move to a desired goal. 
"AICpandapickplace" makes the robot perform a cycle which to mimic the pick and place of objects.

### ROS implementation for franka_ros
The repository also contains a ROS package to use the ROS infrasctucture to control the Panda. The package should be installed within franka_ros. Just clone the package in your franka_ros installation. 

### Video
A video clip showing a comparison between a model reference adaptive controller (MRAC) and the novel active inference controller (AIC) is available at https://youtu.be/Vsb0MzOp_TY

## Troubleshooting

The controller might need re-tuning to avoid some jitter or reduece overshoot for specific applications with ecrtain grippers. This is because the dynamics migth differ too much from real setup to real setup. Single tuning of the control parameters per-joint leads to better responses. As an example, tuning parameters such as learning rates and variances of the last joint(s) might need to be reduced to avoid jittering.  


