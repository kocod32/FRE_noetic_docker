# FRE_noetic_docker

This repository contains a local ROS1 Noetic simulation setup for **FRE Task 4**.

The setup uses Docker because my laptop runs Ubuntu 24.04, while ROS1 Noetic is meant for Ubuntu 20.04. By running ROS Noetic inside a Docker container, I can keep my current Ubuntu version and still work in a ROS1 environment that matches the robot setup.

The repository is only meant for local simulation and development, not as the final robot software.

## What is included

The project contains a Docker-based ROS Noetic environment with a catkin workspace:

```text
catkin_ws/src/
├── fre_task4_description
├── fre_task4_gazebo
├── fre_task4_navigation
├── fre_task4_perception
└── fre_task4_tools
```

## Packages

`fre_task4_description` contains the simplified robot model.  
For now, the robot is only a basic box model with simple TF frames for the body, wheels, lidar, cameras, and tool/nozzle position.

`fre_task4_gazebo` contains the Gazebo simulation files.  
It currently includes a simple world and launch file for spawning the robot in Gazebo.

`fre_task4_navigation` is prepared for navigation-related files.  
Later it can contain configuration for mapping, localization, costmaps, and path planning.

`fre_task4_perception` is prepared for sensor-processing code.  
Later it can contain camera, depth camera, lidar, and detection logic.

`fre_task4_tools` is prepared for helper scripts.  
This can later include test tools, field generators, debug nodes, or RViz markers.

## Basic usage

Open the project in VS Code and reopen it in the Dev Container.

Inside the container:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

To show the robot in RViz:

```bash
roslaunch fre_task4_description display_robot.launch
```

To spawn the robot in Gazebo:

```bash
roslaunch fre_task4_gazebo empty_world.launch
```