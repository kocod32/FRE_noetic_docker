# What is inside the FRE_noetic_docker project

This project is prepared as a basic development environment for working on FRE Task 4 with ROS1 Noetic. Since my laptop runs Ubuntu 24.04, ROS Noetic is not installed directly on the system. Instead, it runs inside a Docker container. This way I can keep using a newer Ubuntu version, while still having an environment that is compatible with the robot, where ROS1 is currently used.

The main purpose of this repository is to have a clean starting point for simulating the robot in Gazebo, visualizing the robot model in RViz, and later developing localization, mapping, sensor processing, and algorithms for Task 4.

## Basic idea

The project uses a Docker image with a ROS Noetic environment. Inside the container there is a `catkin_ws` workspace, which contains ROS packages for the robot description, Gazebo simulation, navigation, perception, and helper tools.

On my laptop I open the project in VS Code and then reopen it as a Dev Container. This means that VS Code works directly inside the Docker container, where ROS1 commands such as `catkin_make`, `roslaunch`, `rostopic`, `rosrun`, and others are already available.

## Project structure

The basic project structure is:

```text
FRE_noetic_docker/
├── Dockerfile
├── docker-compose.yml
├── run.sh
├── README.md
├── .devcontainer/
│   └── devcontainer.json
├── catkin_ws/
│   └── src/
│       ├── fre_task4_description/
│       ├── fre_task4_gazebo/
│       ├── fre_task4_navigation/
│       ├── fre_task4_perception/
│       └── fre_task4_tools/
├── bags/
└── maps/
```

The `catkin_ws/src` folder contains all ROS packages. The `build` and `devel` folders are created automatically when building the workspace with `catkin_make`, but they are not part of the GitHub repository because they are generated files.

## Docker part

The `Dockerfile` defines the environment that is built. It is based on a ROS Noetic desktop environment, with additional packages that will be useful for simulation, navigation, mapping, and perception.

The `docker-compose.yml` file defines how the container is started. The container needs access to the graphical interface because RViz and Gazebo are used. Because of that, the setup includes X11 mounts, access to `/dev`, host networking, and other required settings.

The `run.sh` file is a simple startup script. It can be used when I want to start the container manually from the terminal.

## VS Code Dev Container

The `.devcontainer` folder contains the VS Code container setup. With this, I can open the project directly inside the Docker container. This is useful because the VS Code terminal then already uses the ROS Noetic environment.

The typical workflow is:

```bash
cd ~/workspaces/fre_noetic_docker
code .
```

Then in VS Code I select:

```text
Dev Containers: Reopen in Container
```

When the project is opened inside the container, the terminal points to:

```text
/root/catkin_ws
```

This means I can directly use ROS1 commands from the VS Code terminal.

## Catkin workspace

ROS1 uses a `catkin_ws` workspace, not a `colcon` workspace like ROS2. The main folder for ROS packages is:

```text
catkin_ws/src/
```

The workspace is built with:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

After each build, `source devel/setup.bash` has to be run so that ROS can detect new packages and changes.

## Packages in the project

### fre_task4_description

This package contains the robot description. For now, the robot is modeled as a simplified box, because in this phase I do not need a detailed CAD model yet.

The package contains:

```text
urdf/
launch/
meshes/
rviz/
config/
```

The most important file is the Xacro/URDF robot model. It defines the basic parts of the robot:

- the main robot body,
- wheels,
- lidar,
- front camera,
- downward-facing camera,
- treatment tool or nozzle.

This model is used for visualization in RViz and for spawning the robot in Gazebo.

The RViz display is started with:

```bash
roslaunch fre_task4_description display_robot.launch
```

In RViz, the `Fixed Frame` is set to:

```text
base_footprint
```

Then `RobotModel` is added so that the robot is shown.

### fre_task4_gazebo

This package contains the Gazebo simulation. Currently, it includes a basic empty world where the robot is spawned.

The main folders are:

```text
launch/
worlds/
config/
```

The Gazebo simulation is started with:

```bash
roslaunch fre_task4_gazebo empty_world.launch
```

The launch file loads the robot description, starts Gazebo, and spawns the robot into the simulation.

At the moment, this is mainly a basic test to check that the robot model works correctly in simulation.

### fre_task4_navigation

This package is prepared for the navigation part. Later, it will contain the configuration for navigation, localization, mapping, and robot movement around the field.

This package will likely include:

- `move_base` configuration,
- costmap settings,
- local and global planner configuration,
- AMCL or another localization method,
- navigation launch files.

For now, the package exists as a prepared structure, so files can be added later.

### fre_task4_perception

This package is intended for perception and sensor processing. Since the robot will use cameras and lidar, this package will later contain code for processing sensor data.

This will likely include:

- processing images from depth cameras,
- converting depth images into useful information,
- detecting edges, poles, or markers,
- detecting areas that need treatment.

For now, the package is prepared as a starting point.

### fre_task4_tools

This package is meant for helper scripts. These are scripts that are not directly part of navigation or perception, but help with testing and debugging.

Later, this package can include:

- random field or polygon generation,
- testing scripts,
- RViz markers,
- result-saving tools,
- simple debug nodes.

## How the project is used

First, I open the project in VS Code and start it as a Dev Container. Then, inside the container terminal, I build the workspace:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

To check only the robot model in RViz:

```bash
roslaunch fre_task4_description display_robot.launch
```

To start the Gazebo simulation:

```bash
roslaunch fre_task4_gazebo empty_world.launch
```

This opens Gazebo and spawns the basic robot model into the world.

## What currently works

At the moment, the following parts work:

- Docker environment for ROS Noetic,
- VS Code Dev Container,
- catkin workspace,
- basic robot model,
- TF connections between robot parts,
- robot visualization in RViz,
- robot spawn in Gazebo.

This means that the basic infrastructure is ready. The next step is to add sensors in Gazebo, mainly lidar and cameras, and then continue with mapping and localization.

## Why the robot is currently only a box

For the current phase, a fully accurate URDF model is not necessary. It is more important that the robot has approximately correct dimensions, correct basic TF frames, and approximate sensor positions.

Because of that, the robot is currently modeled as a simple box. This is enough for testing:

- RViz visualization,
- basic simulation,
- lidar placement,
- camera placement,
- later mapping and navigation.

A more accurate model can be added later if needed.

## Next steps

The next steps are:

1. add robot movement through `/cmd_vel`,
2. add a lidar plugin and the `/scan` topic,
3. add cameras or depth cameras,
4. create a test field in Gazebo,
5. add random generation of poles or field boundaries,
6. start mapping,
7. prepare basic localization,
8. later connect the same concept to the real robot.

The main goal is to first make everything work in simulation, because ideas can be tested faster there. Once the basic process works in simulation, the same concepts can be transferred to the real robot.