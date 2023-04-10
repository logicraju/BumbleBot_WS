#!/usr/bin/env python3

# Rhino Motor Driver (RMCS 2303) - ROS INTERFACE
# ----------------------------------------------
# Note: This node connects the motor driver with the differential_drive ROS package mentioned below:
# https://github.com/jfstepha/differential-drive
"""
    BSD 3-Clause License

    Copyright (c) 2021, Rajesh Subramanian
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import sys
import time
import rospy
from std_msgs.msg import Int16, Int32, UInt16, UInt32, Float32
import motor_driver


class MotorClass:
    def __init__(self, port_name, slave_id, encoder_topic_name, velocity_topic_name):
        self.motor = motor_driver.Controller(port_name, slave_id)
        self.__time_delay = 0.001 # 0.001
        self.encoder_ticks_16bit = self.motor.get_position_16bit()
        self.encoder_ticks_32bit = self.motor.get_position_32bit()
        self.encoder_ticks_32bit_scaled = 0
        self.__previous_velocity = 0
        self.__current_velocity = 0
        self.__status_rotation_direction = ""
        self.__velocity_scale_factor = 19 #25  # 10  # To multiply velocity commands received
        self.__encoder_scale_factor = 0.5  # To reduce encoder values received
        # self.encoder_ticks_pub_16bit = rospy.Publisher(encoder_topic_name + "_16bit", UInt16, queue_size=1)
        self.encoder_ticks_pub_32bit = rospy.Publisher(encoder_topic_name + "_32bit", UInt32, queue_size=1)
        rospy.Subscriber(velocity_topic_name, Float32, self.velocity_callback, queue_size=1)

    def velocity_callback(self, data):
        motor_velocity_radians_per_second = data.data
        scaled_velocity = self.__velocity_scale_factor * motor_velocity_radians_per_second
        self.__current_velocity = scaled_velocity
        self.motor.set_speed(scaled_velocity)

        # Send motor commands only when received velocity has changed
        if self.__current_velocity != self.__previous_velocity:
            if motor_velocity_radians_per_second < 0:
                self.motor.turn_motor_cw()
                self.__status_rotation_direction = "CW"
            elif motor_velocity_radians_per_second > 0:
                self.motor.turn_motor_ccw()
                self.__status_rotation_direction = "CCW"
            else:
                self.motor.brake()
                self.__status_rotation_direction = ""
            self.__previous_velocity = self.__current_velocity

            #time.sleep(self.__time_delay)

    def encoder_transmitter(self):
        #self.encoder_ticks_16bit = self.motor.get_position_16bit()
        self.encoder_ticks_32bit = self.motor.get_position_32bit()
        if (self.encoder_ticks_16bit is not None) and (self.encoder_ticks_32bit is not None):
            # self.encoder_ticks_pub_16bit.publish(int(self.encoder_ticks_16bit))
            self.encoder_ticks_pub_32bit.publish(int(self.encoder_ticks_32bit))
        #time.sleep(self.__time_delay)


if __name__ == '__main__':
    rospy.init_node('motor_ros_interface', anonymous=True)
    left_motor = MotorClass("/dev/left_wheel", 7, "lwheel_ticks", "lwheel_vtarget")
    right_motor = MotorClass("/dev/right_wheel", 7, "rwheel_ticks", "rwheel_vtarget")

    while not rospy.is_shutdown():
        left_motor.encoder_transmitter()
        right_motor.encoder_transmitter()

    # Cleaning Up
    rospy.loginfo("Stopping Motors before exiting ...")
    left_motor.motor.brake()
    right_motor.motor.brake()
    del left_motor
    del right_motor
    sys.exit(0)
