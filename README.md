# FRE Noetic Docker – Task 4 Gazebo Workspace

This repository contains a minimal ROS Noetic + Gazebo Classic workspace for the FRE Task 4 simulation.  
The current setup spawns a randomly generated polygon field with boundary poles and a simple four-wheel Farmbeast-style robot model.

The goal of this branch is to keep the workspace clean, reproducible, and focused on the essentials needed for simulation, control, and later navigation/perception work.

---

## Current status

Working:

- ROS Noetic runs inside Docker.
- Gazebo Classic launches correctly.
- A random Task 4 polygon world can be generated.
- The Farmbeast robot spawns inside the generated field.
- The robot has four visible wheels.
- `gazebo_ros_control` is loaded.
- Joint controllers are loaded and running under the `/farmbeast` namespace.
- `/clock` is published.
- `/farmbeast/joint_states` is published.

Not yet finished:

- Proper high-level driving interface.
- Mapping/localization pipeline.
- Real sensor configuration.
- Autonomous navigation.
- Real robot control parity.

---

## Repository structure

```text
.
├── Dockerfile
├── docker-compose.yml
├── run.sh
├── README.md
├── bags/
├── maps/
├── docker/
└── catkin_ws/
    └── src/
        ├── fre_task4_control/
        ├── fre_task4_description/
        ├── fre_task4_gazebo/
        └── fre_task4_tools/
```

### `fre_task4_description`

Contains the robot model.

Important file:

```text
catkin_ws/src/fre_task4_description/urdf/farmbeast_robot.urdf.xacro
```

This is the current active robot model. It is self-contained and does not depend on `velodyne_description`.

### `fre_task4_control`

Contains controller configuration for `ros_control`.

Important file:

```text
catkin_ws/src/fre_task4_control/config/farmbeast_controllers.yaml
```

Controllers are loaded in the `/farmbeast` namespace.

### `fre_task4_gazebo`

Contains Gazebo worlds and launch files.

Important files:

```text
catkin_ws/src/fre_task4_gazebo/launch/farmbeast_task4_world.launch
catkin_ws/src/fre_task4_gazebo/worlds/generated_task4.world
```

### `fre_task4_tools`

Contains helper scripts.

Important file:

```text
catkin_ws/src/fre_task4_tools/scripts/generate_task4_world.py
```

This script generates a new randomized Task 4 world and robot spawn pose.

---

## Start the Docker container

From the repository root:

```bash
./run.sh
```

Inside the container, the workspace should be available at:

```bash
/root/catkin_ws
```

---

## Build the workspace

Inside the Docker container:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Generate a random Task 4 world

```bash
cd /root/catkin_ws
source devel/setup.bash
rosrun fre_task4_tools generate_task4_world.py
```

Expected output is similar to:

```text
Generated FRE Task 4 world
Polygon vertices: 14
Polygon area: 60.37 m²
Robot spawn: x=-3.03, y=3.21, yaw=-2.01
World file: /root/catkin_ws/src/fre_task4_gazebo/worlds/generated_task4.world
Launch file: /root/catkin_ws/src/fre_task4_gazebo/launch/generated_task4_world.launch
```

The generated world is stored here:

```text
catkin_ws/src/fre_task4_gazebo/worlds/generated_task4.world
```

---

## Launch the Farmbeast simulation

```bash
cd /root/catkin_ws
source devel/setup.bash
roslaunch fre_task4_gazebo farmbeast_task4_world.launch
```

This should open Gazebo and spawn:

- the generated Task 4 polygon,
- boundary poles,
- the Farmbeast robot,
- Gazebo ROS control,
- all configured joint controllers.

---

## Check that the robot spawned correctly

In a second terminal inside the container:

```bash
cd /root/catkin_ws
source devel/setup.bash
rosservice call /gazebo/get_model_properties "model_name: 'fre_robot'"
```

Expected result should include bodies such as:

```text
base_link
left_front_wheel
right_front_wheel
left_back_wheel
right_back_wheel
```

---

## Check Gazebo simulation time

```bash
rostopic list | grep clock
rostopic echo -n 1 /clock
```

Expected:

```text
/clock
```

and a valid clock message.

---

## Check controllers

```bash
rosservice list | grep controller
```

Expected services include:

```text
/farmbeast/controller_manager/list_controller_types
/farmbeast/controller_manager/list_controllers
/farmbeast/controller_manager/load_controller
/farmbeast/controller_manager/reload_controller_libraries
/farmbeast/controller_manager/switch_controller
/farmbeast/controller_manager/unload_controller
```

Then check controller states:

```bash
rosservice call /farmbeast/controller_manager/list_controllers
```

Expected controllers:

```text
joint_state_controller
left_front_z_axis_controller
right_front_z_axis_controller
left_back_z_axis_controller
right_back_z_axis_controller
left_front_wheel_controller
right_front_wheel_controller
left_back_wheel_controller
right_back_wheel_controller
```

All should be in the `running` state.

---

## Check joint states

```bash
rostopic echo -n 1 /farmbeast/joint_states
```

Expected:

```text
name:
  - left_back_wheel_joint
  - left_back_z_axis_joint
  - left_front_wheel_joint
  - left_front_z_axis_joint
  - right_back_wheel_joint
  - right_back_z_axis_joint
  - right_front_wheel_joint
  - right_front_z_axis_joint
```

---

## Send a wheel velocity command

Example command:

```bash
rostopic pub /farmbeast/left_front_wheel_controller/command std_msgs/Float64 "data: 5.0" -r 10
```

For testing all wheels manually, use separate terminals or short commands:

```bash
rostopic pub /farmbeast/left_front_wheel_controller/command std_msgs/Float64 "data: 5.0" -r 10
rostopic pub /farmbeast/right_front_wheel_controller/command std_msgs/Float64 "data: 5.0" -r 10
rostopic pub /farmbeast/left_back_wheel_controller/command std_msgs/Float64 "data: 5.0" -r 10
rostopic pub /farmbeast/right_back_wheel_controller/command std_msgs/Float64 "data: 5.0" -r 10
```

This is still low-level joint control. A proper drive node should later convert `/cmd_vel` into wheel commands.

---

## Important notes

### Do not paste YAML or CMake into the terminal

Files like this:

```yaml
farmbeast:
  joint_state_controller:
    type: joint_state_controller/JointStateController
```

belong inside `.yaml` files, not directly in Bash.

Files like this:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(fre_task4_control)
```

belong inside `CMakeLists.txt`, not directly in Bash.

If these are pasted into the terminal, Bash will return errors such as:

```text
command not found
syntax error near unexpected token
```

That does not necessarily mean the file is wrong. It means the text was executed in the wrong place.

---

## Minimal workspace goal

The current cleaned workspace should focus on:

```text
fre_task4_description   robot URDF/Xacro
fre_task4_control       ros_control controller YAML
fre_task4_gazebo        Gazebo launch/world files
fre_task4_tools         random world generator
```

Packages removed or intentionally excluded from the minimal branch:

```text
fre_task4_navigation
fre_task4_perception
old backup URDF files
unused generated launch files
unused empty launch files
```

Navigation and perception should be added later only when the simulation base is stable.

---

## Recommended cleanup checks

After deleting files, always run:

```bash
grep -R "fre_robot_minimal\|diff_drive_backup\|empty_world\|generated_task4_world\|fre_task4_navigation\|fre_task4_perception" catkin_ws/src README.md
```

Some references may be valid, for example:

```text
gazebo_ros/launch/empty_world.launch
```

because that is the official Gazebo launch file from the installed `gazebo_ros` package.

References to deleted local files should be removed.

---

## Recommended Git workflow

Create a cleanup branch:

```bash
git checkout 4wheel_model
git pull
git checkout -b cleanup_minimal_ws
```

After cleanup:

```bash
git status
git add .
git commit -m "clean minimal task4 simulation workspace"
git push -u origin cleanup_minimal_ws
```

Then create a pull request into `4wheel_model` or merge locally if the branch is tested.

---

## Quick test sequence

Use this sequence after every major cleanup:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash

rosrun xacro xacro src/fre_task4_description/urdf/farmbeast_robot.urdf.xacro > /tmp/farmbeast_test.urdf

rosrun fre_task4_tools generate_task4_world.py

roslaunch fre_task4_gazebo farmbeast_task4_world.launch
```

In another terminal:

```bash
cd /root/catkin_ws
source devel/setup.bash

rosservice call /gazebo/get_model_properties "model_name: 'fre_robot'"
rosservice call /farmbeast/controller_manager/list_controllers
rostopic echo -n 1 /farmbeast/joint_states
```

If all of these work, the minimal simulation base is healthy.

---

## Next development steps

Recommended next steps:

1. Add a simple `/cmd_vel` to wheel-controller bridge.
2. Add a basic odometry publisher.
3. Add a simulated 2D LiDAR or depth sensor.
4. Add SLAM or mapping.
5. Add localization.
6. Add navigation.
7. Replace simplified geometry with a more accurate model only after the control stack is stable.

The priority is not visual realism. The priority is that the simulation behaves close enough to the real robot control structure so the rest of the software stack can be developed against it.
