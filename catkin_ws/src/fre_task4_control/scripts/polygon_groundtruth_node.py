#!/usr/bin/env python3

import math
import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker


class PolygonGroundtruthNode:
    def __init__(self):
        rospy.init_node("polygon_groundtruth_node")

        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_rate = rospy.get_param("~publish_rate", 5.0)

        self.keywords = rospy.get_param(
            "~keywords",
            ["corner_pole"]
        )

        self.min_points = rospy.get_param("~min_points", 3)

        self.points = []

        self.polygon_pub = rospy.Publisher(
            "/task4/polygon_groundtruth",
            PolygonStamped,
            queue_size=10,
            latch=True,
        )

        self.points_marker_pub = rospy.Publisher(
            "/task4/polygon_points_marker",
            Marker,
            queue_size=10,
            latch=True,
        )

        self.line_marker_pub = rospy.Publisher(
            "/task4/polygon_line_marker",
            Marker,
            queue_size=10,
            latch=True,
        )

        self.model_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_callback,
            queue_size=1,
        )

        rospy.loginfo("Polygon groundtruth node started")
        rospy.loginfo("Listening to /gazebo/model_states")
        rospy.loginfo("Publishing /task4/polygon_groundtruth")
        rospy.loginfo("Publishing /task4/polygon_points_marker")
        rospy.loginfo("Publishing /task4/polygon_line_marker")
        rospy.loginfo("Frame ID: %s", self.frame_id)
        rospy.loginfo("Keywords: %s", self.keywords)

    def is_polygon_model(self, name):
        for keyword in self.keywords:
            if keyword in name:
                return True
        return False

    def sort_points_as_polygon(self, raw_points):
        if len(raw_points) < self.min_points:
            return []

        cx = sum(p[0] for p in raw_points) / len(raw_points)
        cy = sum(p[1] for p in raw_points) / len(raw_points)

        sorted_points = sorted(
            raw_points,
            key=lambda p: math.atan2(p[1] - cy, p[0] - cx)
        )

        return sorted_points

    def model_states_callback(self, msg):
        raw_points = []
        names_found = []

        for name, pose in zip(msg.name, msg.pose):
            if self.is_polygon_model(name):
                x = pose.position.x
                y = pose.position.y
                z = 0.0

                raw_points.append((x, y, z))
                names_found.append(name)

        if len(raw_points) < self.min_points:
            return

        self.points = self.sort_points_as_polygon(raw_points)

        rospy.loginfo_throttle(
            3.0,
            "Detected %d polygon boundary points: %s",
            len(self.points),
            names_found,
        )

    def publish_polygon(self):
        if len(self.points) < self.min_points:
            return

        msg = PolygonStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        for x, y, z in self.points:
            p = Point32()
            p.x = x
            p.y = y
            p.z = z
            msg.polygon.points.append(p)

        self.polygon_pub.publish(msg)

    def publish_points_marker(self):
        if len(self.points) < self.min_points:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id

        marker.ns = "task4_polygon_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y, z in self.points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.15
            marker.points.append(p)

        self.points_marker_pub.publish(marker)

    def publish_line_marker(self):
        if len(self.points) < self.min_points:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id

        marker.ns = "task4_polygon_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y, z in self.points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            marker.points.append(p)

        first_x, first_y, first_z = self.points[0]
        p = Point()
        p.x = first_x
        p.y = first_y
        p.z = 0.05
        marker.points.append(p)

        self.line_marker_pub.publish(marker)

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            self.publish_polygon()
            self.publish_points_marker()
            self.publish_line_marker()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = PolygonGroundtruthNode()
        node.run()
    except rospy.ROSInterruptException:
        pass