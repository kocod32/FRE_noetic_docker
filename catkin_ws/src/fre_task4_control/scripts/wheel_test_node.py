#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float64


class FarmbeastWheelTest:
    def __init__(self):
        rospy.init_node("farmbeast_wheel_test_node")

        self.mode = rospy.get_param("~mode", "stop")

        # General wheel command
        self.speed = rospy.get_param("~speed", 0.3)

        # Circle / spin parameters
        self.linear_speed = rospy.get_param("~linear_speed", 0.05)
        self.circle_radius = rospy.get_param("~circle_radius", 3.0)

        # Robot geometry. Use your current approximate robot dimensions.
        self.wheelbase = rospy.get_param("~wheelbase", 0.44)
        self.track_width = rospy.get_param("~track_width", 0.565)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.10)

        # Safety limits
        self.max_wheel_speed = rospy.get_param("~max_wheel_speed", 0.5)
        self.max_steer_angle = rospy.get_param("~max_steer_angle", 0.35)

        # Rate limits
        self.wheel_accel_limit = rospy.get_param("~accel_limit", 0.08)          # rad/s/s
        self.steer_rate_limit = rospy.get_param("~steer_rate_limit", 0.05)      # rad/s

        # For near in-place spin. Larger = safer, smaller = more aggressive.
        self.spin_speed = rospy.get_param("~spin_speed", 0.18)
        self.spin_steer_angle = rospy.get_param("~spin_steer_angle", 0.35)

        self.rate_hz = rospy.get_param("~rate", 30.0)

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

        self.current_wheel = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }

        self.current_steer = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }

        rospy.sleep(1.0)

        rospy.loginfo("Farmbeast wheel test node started")
        rospy.loginfo("Mode: %s", self.mode)
        rospy.loginfo("speed: %.3f rad/s", self.speed)
        rospy.loginfo("linear_speed: %.3f m/s", self.linear_speed)
        rospy.loginfo("circle_radius: %.3f m", self.circle_radius)
        rospy.loginfo("wheelbase: %.3f m", self.wheelbase)
        rospy.loginfo("track_width: %.3f m", self.track_width)
        rospy.loginfo("wheel_radius: %.3f m", self.wheel_radius)
        rospy.loginfo("max_wheel_speed: %.3f rad/s", self.max_wheel_speed)
        rospy.loginfo("max_steer_angle: %.3f rad", self.max_steer_angle)
        rospy.loginfo("wheel_accel_limit: %.3f rad/s/s", self.wheel_accel_limit)
        rospy.loginfo("steer_rate_limit: %.3f rad/s", self.steer_rate_limit)

    def clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def ramp_value(self, current, target, step):
        if target > current + step:
            return current + step
        if target < current - step:
            return current - step
        return target

    def publish_commands(self, target_steer, target_wheel, dt):
        steer_step = self.steer_rate_limit * dt
        wheel_step = self.wheel_accel_limit * dt

        for key in self.current_steer:
            limited_target = self.clamp(
                target_steer[key],
                -self.max_steer_angle,
                self.max_steer_angle,
            )
            self.current_steer[key] = self.ramp_value(
                self.current_steer[key],
                limited_target,
                steer_step,
            )
            self.steer_pubs[key].publish(Float64(self.current_steer[key]))

        for key in self.current_wheel:
            limited_target = self.clamp(
                target_wheel[key],
                -self.max_wheel_speed,
                self.max_wheel_speed,
            )
            self.current_wheel[key] = self.ramp_value(
                self.current_wheel[key],
                limited_target,
                wheel_step,
            )
            self.wheel_pubs[key].publish(Float64(self.current_wheel[key]))

    def stop_cmd(self):
        steer = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }
        wheel = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }
        return steer, wheel

    def forward_cmd(self):
        steer = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }
        wheel = {
            "left_front": self.speed,
            "right_front": self.speed,
            "left_back": self.speed,
            "right_back": self.speed,
        }
        return steer, wheel

    def backward_cmd(self):
        steer = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_back": 0.0,
            "right_back": 0.0,
        }
        wheel = {
            "left_front": -self.speed,
            "right_front": -self.speed,
            "left_back": -self.speed,
            "right_back": -self.speed,
        }
        return steer, wheel

    def circle_cmd(self, direction):
        """
        Gentle 4-wheel-steering circle.
        Front and rear steer opposite directions.
        This is safer than true point-turn.
        """
        sign = 1.0 if direction == "left" else -1.0

        if abs(self.circle_radius) < 0.5:
            radius = 0.5
        else:
            radius = abs(self.circle_radius)

        steer_angle = math.atan(self.wheelbase / radius)
        steer_angle = self.clamp(steer_angle, -self.max_steer_angle, self.max_steer_angle)

        wheel_speed = self.linear_speed / self.wheel_radius
        wheel_speed = self.clamp(wheel_speed, -self.max_wheel_speed, self.max_wheel_speed)

        steer = {
            "left_front": sign * steer_angle,
            "right_front": sign * steer_angle,
            "left_back": -sign * steer_angle,
            "right_back": -sign * steer_angle,
        }

        wheel = {
            "left_front": wheel_speed,
            "right_front": wheel_speed,
            "left_back": wheel_speed,
            "right_back": wheel_speed,
        }

        return steer, wheel

    def spin_cmd(self, direction):
        """
        Safe near-in-place rotation.
        This is intentionally not a perfect mathematical point turn.
        It is softened because Gazebo was collapsing the robot with aggressive commands.
        """
        sign = 1.0 if direction == "left" else -1.0

        a = self.clamp(self.spin_steer_angle, 0.0, self.max_steer_angle)
        v = self.clamp(self.spin_speed, 0.0, self.max_wheel_speed)

        steer = {
            "left_front": -sign * a,
            "right_front": sign * a,
            "left_back": sign * a,
            "right_back": -sign * a,
        }

        wheel = {
            "left_front": -sign * v,
            "right_front": sign * v,
            "left_back": -sign * v,
            "right_back": sign * v,
        }

        return steer, wheel

    def get_target_commands(self):
        if self.mode == "stop":
            return self.stop_cmd()

        if self.mode == "forward":
            return self.forward_cmd()

        if self.mode == "backward":
            return self.backward_cmd()

        if self.mode == "circle_left":
            return self.circle_cmd("left")

        if self.mode == "circle_right":
            return self.circle_cmd("right")

        if self.mode == "spin_left":
            return self.spin_cmd("left")

        if self.mode == "spin_right":
            return self.spin_cmd("right")

        rospy.logwarn_throttle(2.0, "Unknown mode '%s'. Stopping.", self.mode)
        return self.stop_cmd()

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()

            if dt <= 0.0 or dt > 1.0:
                dt = 1.0 / self.rate_hz

            last_time = now

            target_steer, target_wheel = self.get_target_commands()
            self.publish_commands(target_steer, target_wheel, dt)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = FarmbeastWheelTest()
        node.run()
    except rospy.ROSInterruptException:
        pass