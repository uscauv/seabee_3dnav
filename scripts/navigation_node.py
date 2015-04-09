#!/usr/bin/env python

import rospy
import actionlib
import geometry_msgs.msg
import math
import tf
import nav_msgs.msg
import sensor_msgs.msg

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
            rospy.sleep(.5)
        self.motor_stop()

    def motors_run_forwards(self):
        new_msg = geometry_msgs.msg.Twist()
        new_msg.angular.x = 0
        new_msg.angular.y = 0
        new_msg.angular.z = 0
        new_msg.linear.x = .5
        new_msg.linear.y = 0
        new_msg.linear.z = 0
        self.nav_node.pub.publish(new_msg)

    def motor_stop(self):
        new_msg = geometry_msgs.msg.Twist()
        new_msg.angular.x = 0
        new_msg.angular.y = 0
        new_msg.angular.z = 0
        new_msg.linear.x = 0
        new_msg.linear.y = 0
        new_msg.linear.z = 0
        self.nav_node.pub.publish(new_msg)


class SeabeeGoToOrientationServer:

    def __init__(self, nav_node):
        self.nav_node = nav_node
        self.server = actionlib.SimpleActionServer("go_to_orientation", SeabeeGoToOrientationAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        curr_orientation = tf.transformations.euler_from_quaternion(self.nav_node.last_imu_msg.orientation)
        desired_orientation = curr_orientation[2]+goal.delta_theta  # all in radians
        self.seabee_rotate(desired_orientation)
        self.server.set_succeeded()

    def seabee_rotate(self, desired_orientation):
        last_z_orientation = (tf.transformations.euler_from_quaternion(self.nav_node.last_imu_msg.orientation))[2]
        while(abs(desired_orientation-last_z_orientation) > .1):
            if desired_orientation-last_z_orientation < 0:
                self.seabee_rotate_right()
            else:
                self.seabee_rotate_left()
            last_z_orientation = (tf.transformations.euler_from_quaternion(self.nav_node.last_imu_msg.orientation))[2]
        self.seabee_stop_rotate()

    def seabee_rotate_right(self):
        new_msg = geometry_msgs.msg.Twist()
        new_msg.angular.x = 0
        new_msg.angular.y = 0
        new_msg.angular.z = -.2
        new_msg.linear.x = 0
        new_msg.linear.y = 0
        new_msg.linear.z = 0
        self.nav_node.pub.publish(new_msg)

    def seabee_rotate_left(self):
        new_msg = geometry_msgs.msg.Twist()
        new_msg.angular.x = 0
        new_msg.angular.y = 0
        new_msg.angular.z = .2
        new_msg.linear.x = 0
        new_msg.linear.y = 0
        new_msg.linear.z = 0
        self.nav_node.pub.publish(new_msg)

    def seabee_stop_rotate(self):
        new_msg = geometry_msgs.msg.Twist()
        new_msg.angular.x = 0
        new_msg.angular.y = 0
        new_msg.angular.z = 0
        new_msg.linear.x = 0
        new_msg.linear.y = 0
        new_msg.linear.z = 0
        self.nav_node.pub.publish(new_msg)


class NavigationNode(object):

    def initialize(self):
        self.last_odom_msg = None
        self.last_imu_msg = None
        self.pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=50)
        self.odom = rospy.Subscriber("odom", nav_msgs.msg.Odometry, self.odom_callback)
        self.imu = rospy.Subscriber("imu/data", sensor_msgs.msg.Imu, self.imu_callback)
        self.location_server = SeabeeGoToLocationServer(self)
        self.orientation_server = SeabeeGoToOrientationServer(self)
        rospy.spin()

    def odom_callback(self, msg):
        self.last_odom_msg = msg

    def imu_callback(self, msg):
        self.last_imu_msg = msg


if __name__ == "__main__":
    rospy.init_node("navigation")
    nav_node = NavigationNode()
    nav_node.initialize()
