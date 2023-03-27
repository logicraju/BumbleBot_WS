#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


class RobotInteraction:
    # Note: If keys are misaligned, press and hold Home button for few seconds and release
    def joystick_callback(self, data):
        if data.buttons[0] == 1:
            print("Button Y")
            self.music_publisher.publish(True)
        elif data.buttons[1] == 1:
            print("Button B")
            self.button1_press_count = self.button1_press_count + 1
            if self.button1_press_count == 1:
                self.led_publisher.publish("red")
            elif self.button1_press_count == 2:
                self.led_publisher.publish("yellow")
            elif self.button1_press_count == 3:
                self.led_publisher.publish("green")
            elif self.button1_press_count == 4:
                self.led_publisher.publish("all")
            elif self.button1_press_count == 5:
                self.led_publisher.publish("none")
                self.button1_press_count = 0
        elif data.buttons[2] == 1:
            print("Button A")
            self.button2_press_count = self.button2_press_count + 1
            if self.button2_press_count == 1:
                self.lcd_publisher.publish("Please take the item from tray")
            elif self.button2_press_count == 2:
                self.lcd_publisher.publish("idle")
            elif self.button2_press_count == 3:
                self.lcd_publisher.publish("I'm Bumblebot.   Hello :)")
                self.button2_press_count = 0
        elif data.buttons[3] == 1:
            print("Button X")
            self.lcd_publisher.publish("Moving to Goal1")
            time.sleep(1)
            self.goal_publisher.publish(self.goal1)
        elif data.buttons[4] == 1:
            print("Button LB")
            self.lcd_publisher.publish("Moving to Home")
            time.sleep(1)
            self.goal_publisher.publish(self.goal2)
        elif data.buttons[5] == 1:
            print("Button RB")
            self.lcd_publisher.publish("Moving to Goal3")
            time.sleep(1)
            self.goal_publisher.publish(self.goal3)
        elif data.buttons[6] == 1:
            print("Button LT")
            self.lcd_publisher.publish("Moving to Home")
            time.sleep(1)
            self.goal_publisher.publish(self.home)
        elif data.buttons[7] == 1:
            print("Button RT")
            self.led_publisher.publish("none")
        elif data.buttons[8] == 1:
            print("Back")
        elif data.buttons[9] == 1:
            print("Start")

        '''    
        elif data.buttons[10] == 1:
            print("LJ Press")
        elif data.buttons[11] == 1:
            print("RJ Press")
        elif data.buttons[12] == 1:
            print("Home")
        '''

    def android_app_callback(self, data):
        if data.data == "goal1":
            self.lcd_publisher.publish("Moving to Goal1")
            self.goal_publisher.publish(self.goal1)
        elif data.data == "goal2":
            self.lcd_publisher.publish("Moving to Goal2")
            self.goal_publisher.publish(self.goal2)
        elif data.data == "goal3":
            self.lcd_publisher.publish("Moving to Goal3")
            self.goal_publisher.publish(self.goal3)
        elif data.data == "goal4":
            self.lcd_publisher.publish("Moving to Home")
            self.goal_publisher.publish(self.home)

    def __init__(self):
        rospy.init_node('robot_interaction')

        self.button1_press_count = 0
        self.button2_press_count = 0

        self.home = PoseStamped()
        self.goal1 = PoseStamped()
        self.goal2 = PoseStamped()
        self.goal3 = PoseStamped()

        # Goal1
        self.goal1.header.frame_id = 'map'
        self.goal1.header.stamp = rospy.Time.now()
        self.goal1.pose.position.x = 1.82694530487
        self.goal1.pose.position.y = 0.331180095673
        self.goal1.pose.orientation.z = 0.393601604879
        self.goal1.pose.orientation.w = 0.919281119482

        # Goal2
        self.goal2.header.frame_id = 'map'
        self.goal2.header.stamp = rospy.Time.now()
        self.goal2.pose.position.x = 5.73351764679
        self.goal2.pose.position.y = -3.20017242432
        self.goal2.pose.orientation.z = 0.708100173108
        self.goal2.pose.orientation.w = 0.706111991716

        # Goal3
        self.goal3.header.frame_id = 'map'
        self.goal3.header.stamp = rospy.Time.now()
        self.goal3.pose.position.x = 1.37735462189
        self.goal3.pose.position.y = -4.43792200089
        self.goal3.pose.orientation.z = 0.699087137432
        self.goal3.pose.orientation.w = 0.715036484578

        # Home
        self.home.header.frame_id = 'map'
        self.home.header.stamp = rospy.Time.now()
        self.home.pose.position.x = -0.115
        self.home.pose.position.y = -0.029
        self.home.pose.orientation.z = -0.002
        self.home.pose.orientation.w = 1.000

        # Initialize Publisher
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size=1)  # Joystick
        rospy.Subscriber("/move_to_goal", String, self.android_app_callback, queue_size=1)  # Android App
        self.music_publisher = rospy.Publisher('/buzzer', Bool, queue_size=10)
        self.led_publisher = rospy.Publisher('/led', String, queue_size=10)
        self.lcd_publisher = rospy.Publisher('/lcd', String, queue_size=10)


if __name__ == '__main__':
    robot_interaction_obj = RobotInteraction()
    rospy.spin()
