#!/usr/bin/env python

import rospy
import actionlib
import geometry_msgs.msg
import math
import tf

from seabee_3dnav.msg import *


class SeabeeGoToLocationServer:

    def __init__(self, nav_node):
        self.nav_node = nav_node
        self.server = actionlib.SimpleActionServer("go_to_location", SeabeeGoToLocationAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        client = actionlib.SimpleActionClient('go_to_orientation', SeabeeGoToOrientationAction)
        client.wait_for_server()
        desired_orientation = SeabeeGoToOrientationGoal(delta_theta=(math.atan(goal.delta_y/goal.delta_x)))
        client.send_goal_and_wait(desired_orientation)
        self.go_forward_to_location(goal)
        self.server.set_succeeded()

    def go_forward_to_location(self, goal):
        destination = nav_node.last_odom_msg.pose.pose.position.x+math.sqrt(pow(goal.delta_x, 2)+pow(goal.delta_y, 2))
        while destination > nav_node.last_odom_msg.pose.pose.position.x:
            self.motors_run_forwards()

    def motors_run_forwards(self):
        print("Location called, but not implemented")


class SeabeeGoToOrientationServer:

    def __init__(self, nav_node):
        self.nav_node = nav_node
        self.server = actionlib.SimpleActionServer("go_to_orientation", SeabeeGoToOrientationAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        destination_orientation = tf.transformations.euler_from_quaternion(self.nav_node.last_imu_msg.orientation)
        print("Orientation called, but not implemented")
        self.server.set_succeeded()


class NavigationNode(object):

    def initialize(self):
        self.last_odom_msg = None
        self.last_imu_msg = None
        self.pub = self.rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=50)
        self.odom = self.rospy.Subscriber("odom", nav_msgs.msg.Odometry, self.odom_callback)
        self.imu = self.rospy.SubscribeR("imu/data", sensor_msgs.msg.Imu, self.imu_callback)
        self.location_server = SeabeeGoToLocationServer()
        self.orientation_server = SeabeeGoToOrientationServer()
        rospy.spin()

    def odom_callback(self, msg):
        self.last_odom_msg = msg

    def imu_callback(self, msg):
        self.last_imu_msg = msg


if __name__ == "__main__":
    rospy.init_node("navigation")
    nav_node = NavigationNode()
    nav_node.initialize()
