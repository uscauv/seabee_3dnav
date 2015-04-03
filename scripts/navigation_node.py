#!/usr/bin/env python

import rospy


def NavigationNode(object):

    def handle_move_seabee():
        pass


if __name__ == "__main__":
    rospy.init_node("navigation")
    nav_node = NavigationNode()
    rospy.Service("move_seabee", MoveSeabee, nav_node.handle_move_seabee) #MAKE SERVICE TYPE MoveSeabee
