#!/usr/bin/env python3

import time
import rospy
import random
from geometry_msgs.msg import Twist

velocity = Twist()
velocity_publisher = rospy.Publisher("/diffbot_controller/cmd_vel", Twist, queue_size=1)


def send_random_velocity_commands():
    global velocity, velocity_publisher
    random_linear_x = random.uniform(-0.75, 0.75)
    random_angular_z = random.uniform(-0.75, 0.75)
    velocity.linear.x = random_linear_x
    velocity.angular.z = random_angular_z
    velocity_publisher.publish(velocity)


def main():
    # Initializing Node
    rospy.init_node("Motor_Crash_Test")

    # Initializing Twist Message
    global velocity
    velocity.linear.x = 0.0
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = 0.0
    while not rospy.is_shutdown():
        random_time_delay = random.uniform(0.1, 10.0)
        send_random_velocity_commands()
        time.sleep(random_time_delay)


if __name__ == "__main__":
    main()

