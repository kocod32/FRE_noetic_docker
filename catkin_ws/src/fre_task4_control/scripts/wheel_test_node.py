#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float64


class FarmbeastWheelTest:
    def __init__(self):
        rospy.init_node("farmbeast_wheel_test_node")

        self.mode = rospy.get_param("~mode", "forward")

        # For simple modes
        self.speed = rospy.get_param("~speed", 0.5)  # rad/s

        # Geometry from current URDF
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.10)
        self.wheelbase = rospy.get_param("~wheelbase", 0.44)
        self.track_width = rospy.get_param("~track_width", 0.565)

        # Circle mode
        self.circle_radius = rospy.get_param("~circle_radius", 3.0)
        self.linear_speed = rospy.get_param("~linear_speed", 0.15)

        # Safety
        self.max_wheel_speed = rospy.get_param("~max_wheel_speed", 1.5)
        self.max_steer_angle = rospy.get_param("~max_steer_angle", 0.35)  # rad, about 20 deg
        self.accel_limit = rospy.get_param("~accel_limit", 0.25)          # rad/s/s for wheel speed
        self.steer_rate_limit = rospy.get_param("~steer_rate_limit", 0.20) # rad/s

        self.wheel_pubs = {
            "left_front": rospy.Publisher("/farmbeast/left_front_wheel_controller/command", Float64, queue_size=10),
            "right_front": rospy.Publisher("/farmbeast/right_front_wheel_controller/command", Float64, queue_size=10),
            "left_back": rospy.Publisher("/farmbeast/left_back_wheel_controller/command", Float64, queue_size=10),
            "right_back": rospy.Publisher("/farmbeast/right_back_wheel_controller/command", Float64, queue_size=10),
        }

        self.steer_pubs = {
            "left_front": rospy.Publisher("/farmbeast/left_front_z_axis_controller/command", Float64, queue_size=10),
            "right_front": rospy.Publisher("/farmbeast/right_front_z_axis_controller/command", Float64, queue_size=10),
            "left_back": rospy.Publisher("/farmbeast/left_back_z_axis_controller/command", Float64, queue_size=10),
            "right_back": rospy.Publisher("/farmbeast/right_back_z_axis_controller/command", Float64, queue_size=10),
        }

        self.current_wheel_cmd = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }

        self.current_steer_cmd = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }

        rospy.loginfo("Farmbeast wheel test node started")
        rospy.loginfo("Mode: %s", self.mode)
        rospy.loginfo("Wheel radius: %.3f m", self.wheel_radius)
        rospy.loginfo("Wheelbase: %.3f m", self.wheelbase)
        rospy.loginfo("Track width: %.3f m", self.track_width)
        rospy.loginfo("Circle radius: %.3f m", self.circle_radius)
        rospy.loginfo("Linear speed: %.3f m/s", self.linear_speed)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def step_towards(self, current, target, max_step):
        if target > current + max_step:
            return current + max_step
        if target < current - max_step:
            return current - max_step
        return target

    def get_targets(self):
        v = self.speed

        steer_zero = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }

        if self.mode == "stop":
            return steer_zero, {
                "left_front": 0.0,
                "right_front": 0.0,
                "left_back": 0.0,
                "right_back": 0.0,
            }

        if self.mode == "forward":
            return steer_zero, {
                "left_front": v,
                "right_front": v,
                "left_back": v,
                "right_back": v,
            }

        if self.mode == "backward":
            return steer_zero, {
                "left_front": -v,
                "right_front": -v,
                "left_back": -v,
                "right_back": -v,
            }

        if self.mode == "rotate_left":
            # Use very low speed only. This is skid turning and can destabilize Gazebo.
            return steer_zero, {
                "left_front": -v,
                "right_front": v,
                "left_back": -v,
                "right_back": v,
            }

        if self.mode == "rotate_right":
            return steer_zero, {
                "left_front": v,
                "right_front": -v,
                "left_back": v,
                "right_back": -v,
            }

        if self.mode in ["circle_left", "circle_right"]:
            R = abs(self.circle_radius)
            V = self.linear_speed
            L = self.wheelbase
            W = self.track_width
            r = self.wheel_radius

            # Small 4-wheel steering approximation:
            # front wheels steer into the turn, rear wheels steer opposite.
            delta = math.atan(L / R)
            delta = self.clamp(delta, -self.max_steer_angle, self.max_steer_angle)

            if self.mode == "circle_left":
                front_delta = delta
                rear_delta = -delta

                v_left = V * (R - W / 2.0) / R
                v_right = V * (R + W / 2.0) / R

            else:
                front_delta = -delta
                rear_delta = delta

                v_left = V * (R + W / 2.0) / R
                v_right = V * (R - W / 2.0) / R

            wheel_left = v_left / r
            wheel_right = v_right / r

            wheel_left = self.clamp(wheel_left, -self.max_wheel_speed, self.max_wheel_speed)
            wheel_right = self.clamp(wheel_right, -self.max_wheel_speed, self.max_wheel_speed)

            steer_targets = {
                "left_front": front_delta,
                "right_front": front_delta,
                "left_back": rear_delta,
                "right_back": rear_delta,
            }

            wheel_targets = {
                "left_front": wheel_left,
                "right_front": wheel_right,
                "left_back": wheel_left,
                "right_back": wheel_right,
            }

            return steer_targets, wheel_targets

        rospy.logwarn("Unknown mode '%s'. Stopping robot.", self.mode)
        return steer_zero, {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }

    def run(self):
        rate_hz = 50.0
        rate = rospy.Rate(rate_hz)

        wheel_step = self.accel_limit / rate_hz
        steer_step = self.steer_rate_limit / rate_hz

        while not rospy.is_shutdown():
            steer_targets, wheel_targets = self.get_targets()

            for name in self.current_steer_cmd:
                self.current_steer_cmd[name] = self.step_towards(
                    self.current_steer_cmd[name],
                    steer_targets[name],
                    steer_step,
                )

            for name in self.current_wheel_cmd:
                self.current_wheel_cmd[name] = self.step_towards(
                    self.current_wheel_cmd[name],
                    wheel_targets[name],
                    wheel_step,
                )

            for name, pub in self.steer_pubs.items():
                pub.publish(Float64(self.current_steer_cmd[name]))

            for name, pub in self.wheel_pubs.items():
                pub.publish(Float64(self.current_wheel_cmd[name]))

            rate.sleep()


if __name__ == "__main__":
    try:
        node = FarmbeastWheelTest()
        node.run()
    except rospy.ROSInterruptException:
        pass