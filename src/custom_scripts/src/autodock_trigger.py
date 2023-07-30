#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from autodock_core.msg import AutoDockingActionGoal

pub = rospy.Publisher('/autodock_action/goal', AutoDockingActionGoal, queue_size=1)

def autodock_goal_callback(msg):
    global pub
    goal_msg = AutoDockingActionGoal()
    pub.publish(goal_msg)

if __name__ == '__main__':
    rospy.init_node('autodock_goal_publisher', anonymous=True)
    rospy.Subscriber('/start_autodock', Empty, autodock_goal_callback)
    rospy.spin()
