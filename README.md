# FRE Noetic Docker – 4 Wheel Model

Minimal ROS1 Noetic + Gazebo setup for FRE Task 4.

The current branch contains a simple 4-wheel robot model, a generated polygon world with boundary poles, basic wheel test control, and a ground-truth polygon publisher for RViz.

This is not a finished SLAM solution yet. For now, the goal is to have the simulated robot, world generation, wheel motion, and polygon visualization working reliably.

---

## 1. Start the Docker container

From the repository root on the host machine:

```bash
./run.sh
```

Inside the container, go to the catkin workspace:

```bash
cd /root/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

---

## 2. Build the workspace

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

If something behaves strangely after editing URDF, launch files, or scripts, rebuild and source again:

```bash
catkin_make
source devel/setup.bash
```

---

## 3. Generate a new Task 4 world

This creates a random polygon area and places the boundary poles.

```bash
rosrun fre_task4_tools generate_task4_world.py
```

Output should show something like:

```text
Generated FRE Task 4 world
Polygon vertices: 8
Polygon area: ... m²
Suggested robot spawn: x=..., y=..., yaw=...
World file: /root/catkin_ws/src/fre_task4_gazebo/worlds/generated_task4.world
```

---

## 4. Launch Gazebo world and robot

```bash
roslaunch fre_task4_gazebo farmbeast_task4_world.launch
```

This starts:

- Gazebo world
- Farmbeast robot model
- `gazebo_ros_control`
- wheel controllers
- steering controllers
- robot state publisher

Useful note: if `gzclient` crashes but Gazebo server still runs, the simulation may still be alive. You can reopen GUI separately with:

```bash
gzclient
```

---

## 5. Test robot movement

Open a new terminal inside the same Docker container.

Always source first:

```bash
cd /root/catkin_ws
source devel/setup.bash
```

### Drive forward

```bash
rosrun fre_task4_control wheel_test_node.py \
  _mode:=forward \
  _speed:=0.3
```

### Stop robot

```bash
rosrun fre_task4_control wheel_test_node.py \
  _mode:=stop
```

### Gentle circle

This is the most stable turning test at the moment.

```bash
rosrun fre_task4_control wheel_test_node.py \
  _mode:=circle_left \
  _circle_radius:=3.0 \
  _linear_speed:=0.05 \
  _wheelbase:=0.44 \
  _track_width:=0.565 \
  _wheel_radius:=0.10 \
  _max_wheel_speed:=0.6 \
  _accel_limit:=0.08 \
  _steer_rate_limit:=0.05 \
  _max_steer_angle:=0.35
```

### Spin test

This is only a rough test. It is useful for checking rotation, but the 4-wheel steering model can become unstable if the values are too aggressive.

```bash
rosrun fre_task4_control wheel_test_node.py \
  _mode:=spin_left \
  _spin_speed:=0.12 \
  _spin_steer_angle:=0.25 \
  _max_wheel_speed:=0.4 \
  _accel_limit:=0.05 \
  _steer_rate_limit:=0.04 \
  _max_steer_angle:=0.35
```

If the robot starts folding into itself or Gazebo becomes unstable, reduce:

- `_spin_speed`
- `_spin_steer_angle`
- `_max_wheel_speed`

---

## 6. Publish polygon ground truth

This node reads the pole positions from Gazebo and publishes the polygon points.

Open a new terminal:

```bash
cd /root/catkin_ws
source devel/setup.bash
rosrun fre_task4_control polygon_groundtruth_node.py
```

Expected output:

```text
Polygon groundtruth node started
Listening to /gazebo/model_states
Publishing /task4/polygon_groundtruth
Publishing /task4/polygon_points_marker
Publishing /task4/polygon_line_marker
Detected ... polygon boundary points
```

---

## 7. View polygon in RViz

Open another terminal:

```bash
cd /root/catkin_ws
source devel/setup.bash
rviz
```

In RViz:

1. Set **Fixed Frame** to:

```text
world
```

2. Add a **Marker** display.

3. Set marker topic to:

```text
/task4/polygon_points_marker
```

4. Add another **Marker** display.

5. Set marker topic to:

```text
/task4/polygon_line_marker
```

You should see the detected pole points and the polygon outline.

---

## Main ROS topics

### Gazebo

```text
/gazebo/model_states
```

Contains all model poses in Gazebo. The polygon ground-truth node reads corner pole positions from here.

### Polygon output

```text
/task4/polygon_groundtruth
/task4/polygon_points_marker
/task4/polygon_line_marker
```

`/task4/polygon_groundtruth` publishes the polygon as geometry messages.

The marker topics are for RViz visualization.

### Wheel velocity commands

```text
/farmbeast/left_front_wheel_controller/command
/farmbeast/right_front_wheel_controller/command
/farmbeast/left_back_wheel_controller/command
/farmbeast/right_back_wheel_controller/command
```

### Steering position commands

```text
/farmbeast/left_front_z_axis_controller/command
/farmbeast/right_front_z_axis_controller/command
/farmbeast/left_back_z_axis_controller/command
/farmbeast/right_back_z_axis_controller/command
```

---

## Useful debug commands

List topics:

```bash
rostopic list
```

Check polygon output:

```bash
rostopic echo /task4/polygon_groundtruth
```

Check Gazebo model states:

```bash
rostopic echo /gazebo/model_states
```

Check controllers:

```bash
rosservice call /farmbeast/controller_manager/list_controllers
```

Check TF frames:

```bash
rosrun tf view_frames
```

---

## Current status

Working:

- Docker ROS1 Noetic environment
- Generated Task 4 polygon world
- 4-wheel Farmbeast model in Gazebo
- Basic forward motion
- Gentle circle motion
- Rough spin test
- Polygon ground-truth extraction from Gazebo
- RViz polygon visualization

Still rough:

- Robot physics during aggressive turning
- Near-in-place rotation stability
- Real SLAM is not implemented yet
- Pole detection is currently ground-truth based, not sensor based

---

## What to do next

Next useful step is not to tune the robot visually anymore. The next step should be adding a simple sensor-based pipeline.

Recommended order:

1. Add a 2D LiDAR or depth sensor to the robot URDF.
2. Publish sensor data in ROS.
3. Visualize the scan in RViz.
4. Detect boundary poles from sensor data.
5. Compare detected pole positions with `/task4/polygon_groundtruth`.
6. Only after that, try real SLAM or mapping.

For a quick next milestone, make the robot detect the poles with a simulated LiDAR and publish detected pole markers in RViz.
