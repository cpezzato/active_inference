# Active_inference
Minimal version of an active inference controller applied to a 7-DOF robot arm.

## C++ scripts
This repo contains the minimal implementation of active infernce to control the real Franka Emika Panda 7-DOF robot arm. 
The control is going to be in the joint space using torque commands and position/velocity sensors.

"AICpanda.cpp" makes the robot move to a desired goal. 
"AICpandapickplace" makes the robot perform a cycle which will be completed with the pick and place of objects later on.

