# ROS2 MRAC Lower-Limb Exoskeleton

This project implements a Model Reference Adaptive Control (MRAC) framework for an 8-DOF lower-limb exoskeleton using ROS2.

## Project Overview

The system contains a lower-limb exoskeleton model with right and left leg joints. The controller tracks desired joint trajectories and generates adaptive control commands for analysis and visualization.

## Features

* ROS2-based MRAC controller
* 8-DOF lower-limb exoskeleton model
* Desired joint trajectory tracking
* Adaptive parameter update
* RViz visualization
* CSV-based trajectory and error analysis

## Main Folders

* src - controller source code
* include - header files
* urdf - robot description
* meshes - STL mesh files
* launch - ROS2 launch files
* rviz - RViz configuration
* scripts - analysis scripts

## Joint Names

* right_joint_1
* right_joint_2
* right_joint_3
* right_joint_4
* left_joint_1
* left_joint_2
* left_joint_3
* left_joint_4

## Build Instructions

cd ~/ros2_ws
colcon build --packages-select adaptive_controller
source install/setup.bash

## Run Instructions

ros2 launch adaptive_controller display.launch.py

ros2 run adaptive_controller mrac_controller

## Output

The project generates desired trajectory, measured joint response, tracking error, control torque, and adaptive parameter data for performance analysis.

## Author

Bipasha Deka
B.Tech Mechanical Engineering
Tezpur University
