#! /usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from copy import deepcopy
import tf

odom = Odometry()
def OdomCB(msg:Odometry)->None:
    global odom
    odom = msg

if __name__ == "__main__":
    rospy.init_node('my_nav', anonymous=False)
    odom_sub = rospy.Subscriber('/odom', Odometry, OdomCB)

    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    as_ = actionlib.SimpleActionServer('move_base', MoveBaseAction, )

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.position.y = 0.5
    goal.target_pose.pose.position.z = 0.01

    print(goal)
    ac.send_goal(goal)

    # ac.send_goal_and_wait(goal, rospy.Duration(10.0))
    rospy.spin()
