#!/usr/bin/env python3

import math
import random
from pathlib import Path


MIN_AREA = 60.0
MAX_AREA = 70.0

MIN_VERTICES = 8
MAX_VERTICES = 15

WORLD_OUT = Path("/root/catkin_ws/src/fre_task4_gazebo/worlds/generated_task4.world")


def polygon_area(points):
    area = 0.0
    n = len(points)

    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        area += x1 * y2 - x2 * y1

    return abs(area) / 2.0


def polygon_centroid(points):
    x_sum = sum(p[0] for p in points)
    y_sum = sum(p[1] for p in points)

    return x_sum / len(points), y_sum / len(points)


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)

    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        intersect = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi
        )

        if intersect:
            inside = not inside

        j = i

    return inside


def generate_polygon():
    vertex_count = random.randint(MIN_VERTICES, MAX_VERTICES)

    angles = sorted(
        random.uniform(0, 2 * math.pi)
        for _ in range(vertex_count)
    )

    points = []
    for angle in angles:
        radius = random.uniform(3.0, 5.5)
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        points.append((x, y))

    # Move centroid near origin.
    cx, cy = polygon_centroid(points)
    points = [(x - cx, y - cy) for x, y in points]

    # Scale polygon to target area.
    current_area = polygon_area(points)
    target_area = random.uniform(MIN_AREA, MAX_AREA)
    scale = math.sqrt(target_area / current_area)

    points = [(x * scale, y * scale) for x, y in points]

    return points


def random_pose_inside_polygon(polygon):
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    for _ in range(10000):
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)

        if point_in_polygon((x, y), polygon):
            yaw = random.uniform(-math.pi, math.pi)
            return x, y, yaw

    raise RuntimeError("Could not sample robot pose inside polygon.")


def make_cylinder_model(name, x, y, z, radius, length, rgba):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{length}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{length}</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
"""


def make_sphere_model(name, x, y, z, radius, rgba):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>{radius}</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>{radius}</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>
"""


def make_boundary_segment(name, p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    mx = (x1 + x2) / 2.0
    my = (y1 + y2) / 2.0

    dx = x2 - x1
    dy = y2 - y1

    length = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx)

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{mx:.3f} {my:.3f} 0.015 0 0 {yaw:.3f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{length:.3f} 0.035 0.03</size>
            </box>
          </geometry>
          <material>
            <ambient>0.1 0.8 0.1 1</ambient>
            <diffuse>0.1 0.8 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def create_world(polygon):
    models = ""

    for i, (x, y) in enumerate(polygon):
        models += make_cylinder_model(
            name=f"corner_pole_{i}",
            x=x,
            y=y,
            z=0.275,
            radius=0.03,
            length=0.55,
            rgba="0.05 0.05 0.05 1",
        )

        models += make_sphere_model(
            name=f"reflector_{i}",
            x=x,
            y=y,
            z=0.6,
            radius=0.07,
            rgba="1.0 0.6 0.05 1",
        )

    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        models += make_boundary_segment(f"debug_boundary_{i}", p1, p2)

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="generated_task4_world">

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    {models}

  </world>
</sdf>
"""


def main():
    WORLD_OUT.parent.mkdir(parents=True, exist_ok=True)

    polygon = generate_polygon()
    area = polygon_area(polygon)
    robot_x, robot_y, robot_yaw = random_pose_inside_polygon(polygon)

    world_text = create_world(polygon)
    WORLD_OUT.write_text(world_text)

    print("Generated FRE Task 4 world")
    print(f"Polygon vertices: {len(polygon)}")
    print(f"Polygon area: {area:.2f} m²")
    print(f"Suggested robot spawn: x={robot_x:.2f}, y={robot_y:.2f}, yaw={robot_yaw:.2f}")
    print(f"World file: {WORLD_OUT}")
    print("")
    print("Launch manually with:")
    print("roslaunch fre_task4_gazebo farmbeast_task4_world.launch")


if __name__ == "__main__":
    main()