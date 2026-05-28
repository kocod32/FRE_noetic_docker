#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from std_msgs.msg import Float64
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class FarmbeastFakeDrive:
    def __init__(self):
        rospy.init_node("farmbeast_fake_drive_node")

        self.model_name = rospy.get_param("~model_name", "fre_robot")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")

        self.mode = rospy.get_param("~mode", "spin_left")

        self.rate_hz = rospy.get_param("~rate_hz", 50.0)

        # Safe fake-motion parameters
        self.linear_speed = rospy.get_param("~linear_speed", 0.05)      # m/s
        self.angular_speed = rospy.get_param("~angular_speed", 0.25)    # rad/s
        self.circle_radius = rospy.get_param("~circle_radius", 1.5)     # m

        # Limits so motion is never violent
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.15)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.35)

        # Optional visual wheel/steering commands.
        # These are only cosmetic; they should stay gentle.
        self.publish_visual_wheels = rospy.get_param("~publish_visual_wheels", True)
        self.visual_wheel_speed = rospy.get_param("~visual_wheel_speed", 0.25)
        self.visual_steer_angle = rospy.get_param("~visual_steer_angle", 0.0)

        # Initial pose fallback
        self.x = rospy.get_param("~x0", 0.0)
        self.y = rospy.get_param("~y0", 0.0)
        self.z = rospy.get_param("~z0", 0.25)
        self.yaw = rospy.get_param("~yaw0", 0.0)

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

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

        rospy.loginfo("Waiting for Gazebo services...")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")

        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.try_read_initial_pose()

        rospy.loginfo("Farmbeast fake drive node started")
        rospy.loginfo("Model name: %s", self.model_name)
        rospy.loginfo("Mode: %s", self.mode)
        rospy.loginfo("Initial pose: x=%.3f y=%.3f z=%.3f yaw=%.3f", self.x, self.y, self.z, self.yaw)
        rospy.loginfo("linear_speed=%.3f m/s angular_speed=%.3f rad/s circle_radius=%.3f m",
                      self.linear_speed, self.angular_speed, self.circle_radius)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def try_read_initial_pose(self):
        try:
            response = self.get_model_state(self.model_name, "world")
            if response.success:
                pose = response.pose
                self.x = pose.position.x
                self.y = pose.position.y
                self.z = pose.position.z

                q = pose.orientation
                roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.yaw = yaw

                rospy.loginfo("Read initial pose from Gazebo.")
            else:
                rospy.logwarn("Could not read model pose from Gazebo. Using fallback pose.")
        except Exception as e:
            rospy.logwarn("GetModelState failed: %s", str(e))

    def get_motion_command(self):
        v = self.clamp(self.linear_speed, -self.max_linear_speed, self.max_linear_speed)
        w = self.clamp(self.angular_speed, -self.max_angular_speed, self.max_angular_speed)

        if self.mode == "stop":
            return 0.0, 0.0

        if self.mode == "forward":
            return v, 0.0

        if self.mode == "backward":
            return -v, 0.0

        if self.mode == "spin_left":
            return 0.0, abs(w)

        if self.mode == "spin_right":
            return 0.0, -abs(w)

        if self.mode == "circle_left":
            radius = max(abs(self.circle_radius), 0.2)
            omega = abs(v) / radius
            omega = self.clamp(omega, 0.0, self.max_angular_speed)
            return abs(v), omega

        if self.mode == "circle_right":
            radius = max(abs(self.circle_radius), 0.2)
            omega = abs(v) / radius
            omega = self.clamp(omega, 0.0, self.max_angular_speed)
            return abs(v), -omega

        rospy.logwarn_throttle(2.0, "Unknown mode '%s'. Stopping.", self.mode)
        return 0.0, 0.0

    def publish_cosmetic_wheel_commands(self, v, w):
        if not self.publish_visual_wheels:
            return

        steer = self.clamp(self.visual_steer_angle, -0.25, 0.25)
        wheel_speed = self.clamp(self.visual_wheel_speed, -0.4, 0.4)

        if self.mode == "stop":
            wheel_targets = {
                "left_front": 0.0,
                "right_front": 0.0,
                "left_back": 0.0,
                "right_back": 0.0,
            }
            steer_targets = {
                "left_front": 0.0,
                "right_front": 0.0,
                "left_back": 0.0,
                "right_back": 0.0,
            }

        elif self.mode == "spin_left":
            wheel_targets = {
                "left_front": -wheel_speed,
                "right_front": wheel_speed,
                "left_back": -wheel_speed,
                "right_back": wheel_speed,
            }
            steer_targets = {
                "left_front": 0.0,
                "right_front": 0.0,
                "left_back": 0.0,
                "right_back": 0.0,
            }

        elif self.mode == "spin_right":
            wheel_targets = {
                "left_front": wheel_speed,
                "right_front": -wheel_speed,
                "left_back": wheel_speed,
                "right_back": -wheel_speed,
            }
            steer_targets = {
                "left_front": 0.0,
                "right_front": 0.0,
                "left_back": 0.0,
                "right_back": 0.0,
            }

        elif self.mode == "circle_left":
            wheel_targets = {
                "left_front": 0.7 * wheel_speed,
                "right_front": wheel_speed,
                "left_back": 0.7 * wheel_speed,
                "right_back": wheel_speed,
            }
            steer_targets = {
                "left_front": steer,
                "right_front": steer,
                "left_back": -steer,
                "right_back": -steer,
            }

        elif self.mode == "circle_right":
            wheel_targets = {
                "left_front": wheel_speed,
                "right_front": 0.7 * wheel_speed,
                "left_back": wheel_speed,
                "right_back": 0.7 * wheel_speed,
            }
            steer_targets = {
                "left_front": -steer,
                "right_front": -steer,
                "left_back": steer,
                "right_back": steer,
            }

        else:
            sign = 1.0 if v >= 0.0 else -1.0
            wheel_targets = {
                "left_front": sign * wheel_speed,
                "right_front": sign * wheel_speed,
                "left_back": sign * wheel_speed,
                "right_back": sign * wheel_speed,
            }
            steer_targets = {
                "left_front": 0.0,
                "right_front": 0.0,
                "left_back": 0.0,
                "right_back": 0.0,
            }

        for name, pub in self.wheel_pubs.items():
            pub.publish(Float64(wheel_targets[name]))

        for name, pub in self.steer_pubs.items():
            pub.publish(Float64(steer_targets[name]))

    def set_gazebo_pose(self, v, w):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"

        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation = Quaternion(qx, qy, qz, qw)

        state.twist.linear.x = v
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = w

        try:
            self.set_model_state(state)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "SetModelState failed: %s", str(e))

    def publish_odom_and_tf(self, v, w):
        now = rospy.Time.now()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = Quaternion(qx, qy, qz, qw)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.z
        tf_msg.transform.rotation = Quaternion(qx, qy, qz, qw)

        self.tf_broadcaster.sendTransform(tf_msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now

            if dt <= 0.0 or dt > 0.2:
                dt = 1.0 / self.rate_hz

            v, w = self.get_motion_command()

            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += w * dt
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            self.set_gazebo_pose(v, w)
            self.publish_odom_and_tf(v, w)
            self.publish_cosmetic_wheel_commands(v, w)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = FarmbeastFakeDrive()
        node.run()
    except rospy.ROSInterruptException:
        pass