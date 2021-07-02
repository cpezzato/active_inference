# Active_inference
This repository contains the C++ implementation of classical active inference and unbiased active inference for robot control. For simulation purposes, this repo can be handy instead https://github.com/cpezzato/panda_simulation

### How to cite this work
This package was developed to support the theoretical results of these works:

- Pezzato C., Ferrari, R., Henrandez C., *A Novel Adaptive Controller for Robot Manipulators Based on Active Inference*, IEEE Robotics and Automation Letters, 2020. 
- Baioumy, M., Pezzato C., Ferrari, R., Henrandez C., Hawes, N., *Fault-tolerant Control of Robot Manipulators with Sensory Faults using Unbiased Active Inference*, ECC, 2021. 
- 
If you found this controllers useful, please consider citing the papers above. 

### ROS implementation for franka_ros
The repository also contains a ROS package to use the ROS infrasctucture to control the Panda. The package should be installed within franka_ros. Just clone the package in your franka_ros installation. To run the controllers simply:

`roslaunch franka_custom_controllers AIC_controller.launch robot_ip:=YOURIP (i.e. 172.16.0.2)`

or for the unbiased:

`roslaunch franka_custom_controllers uAIC_controller.launch robot_ip:=YOURIP (i.e. 172.16.0.2)`

### Expected behavior
The robot will initially move to a central position, then wait for joint space goals to be published in the topic */GoalPositions*.

