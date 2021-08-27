#!/usr/bin/env python

# Gets feedback from Move Base and triggers arduino functions

from datetime import datetime
import time
import rospy
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped


class DeliveryApplication:
    def push_button_callback(self, data):
        if data.data:  # Button Pressed
            if not self.first_press_detected:
                self.first_press_time = datetime.now()
                self.first_press_detected = True
            self.goal = self.goals[self.press_count]
            self.lcd_publisher.publish("")
            if self.press_count == 0:
                self.lcd_publisher.publish("Target: HOME")
            else:
                self.lcd_publisher.publish("Target: GOAL-" + str(self.press_count))
            self.press_count = (self.press_count + 1) % len(self.goals)
            time.sleep(1)

        if self.first_press_detected:
            time_elapsed = (datetime.now() - self.first_press_time).total_seconds()
            if time_elapsed > 10:
                self.first_press_detected = False
                self.press_count = 0
                self.goal_publisher.publish(self.goal)
                self.lcd_publisher.publish("Goal Selection    Complete")

    def callback(self, data):
        if not self.first_press_detected:
                if len(data.status_list) > 0:
                    current_status = data.status_list[len(data.status_list)-1].status  # Get the status from the latest message
                    print("Status of last goal: " + str(current_status))
                    if current_status == 1:  # ACTIVE
                        self.led_publisher.publish("yellow")
                        self.lcd_publisher.publish("Goal Received")
                        self.music_triggered = False
                    elif current_status == 3:  # SUCCEEDED
                        self.led_publisher.publish("green")
                        self.lcd_publisher.publish("Goal Reached")
                        time.sleep(1)
                        if not self.music_triggered:
                            self.music_publisher.publish(True)
                            self.music_triggered = True
                            self.music_publisher.publish(False)
                    else:  # OTHER
                        self.led_publisher.publish("red")
                        self.lcd_publisher.publish("Error...")

    def __init__(self):
        rospy.init_node('arduino_integration_node')

        # Goals
        self.home = PoseStamped()
        self.goal1 = PoseStamped()
        self.goal2 = PoseStamped()
        self.goal3 = PoseStamped()

        # Goal1
        self.goal1.header.frame_id = 'map'
        self.goal1.header.stamp = rospy.Time.now()
        self.goal1.pose.position.x = 7.80939388275
        self.goal1.pose.position.y = -2.87791061401
        self.goal1.pose.orientation.z = 0.999088019833
        self.goal1.pose.orientation.w = 0.0426981103338

        # Goal2
        self.goal2.header.frame_id = 'map'
        self.goal2.header.stamp = rospy.Time.now()
        self.goal2.pose.position.x = 4.8556022644
        self.goal2.pose.position.y = -6.80550956726
        self.goal2.pose.orientation.z = 0.717047815957
        self.goal2.pose.orientation.w = 0.697023980672

        # Goal3
        self.goal3.header.frame_id = 'map'
        self.goal3.header.stamp = rospy.Time.now()
        self.goal3.pose.position.x = 5.53890752792
        self.goal3.pose.position.y = 0.579120635986
        self.goal3.pose.orientation.z = -0.708675695394
        self.goal3.pose.orientation.w = 0.705534378155

        # Home
        self.home.header.frame_id = 'map'
        self.home.header.stamp = rospy.Time.now()
        self.home.pose.position.x = 0.0399286746979
        self.home.pose.position.y = -0.071713924408
        self.home.pose.orientation.z = -0.00775012578072
        self.home.pose.orientation.w = 0.999969967324

        self.goals = [self.home, self.goal1, self.goal2, self.goal3]

        self.first_press_time = datetime.now()
        self.press_count = 0
        self.goal = self.home
        self.music_triggered = False
        self.current_status = 0  # Default
        self.first_press_detected = False
        self.led_publisher = rospy.Publisher('/led', String, queue_size=10)
        self.music_publisher = rospy.Publisher('/buzzer', Bool, queue_size=10)
        self.lcd_publisher = rospy.Publisher('/lcd', String, queue_size=10)
        rospy.Subscriber("/push_button", Bool, self.push_button_callback, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback, queue_size=1)
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        # Performing Initial Operations
        time.sleep(5)
        self.led_publisher.publish("all")  # Blink all LEDs
        self.lcd_publisher.publish("Starting...")
        time.sleep(5)
        self.led_publisher.publish("none")  # Blink all LEDs
        self.lcd_publisher.publish("Ready")
        self.music_publisher.publish(False)


if __name__ == "__main__":
    delivery_app = DeliveryApplication()
    rospy.spin()
