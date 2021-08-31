#!/usr/bin/env python3

# Rhino Motor Driver (RMCS 2303) - Basic Modbus Communication
# -----------------------------------------------------------

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

import time
import traceback
import minimalmodbus as modbus
import rhino_params as rhino


class Controller:
    def __init__(self, port_name, slave_address):
        # Parameters
        self.__instrument = modbus.Instrument(port_name, slave_address, modbus.MODE_ASCII)
        self.__instrument.serial.baudrate = 9600
        self.__instrument.serial.parity = modbus.serial.PARITY_NONE
        self.__instrument.bytesize = 8
        self.__instrument.stopbits = 1
        self.__instrument.timeout = 5  # seconds
        self.__instrument.write_timeout = 5  # seconds
        self.__instrument.clear_buffers_before_each_transaction = True
        # self.__instrument.close_port_after_each_call = True
        self.__time_delay = 0.001 #0.001 # default: 1 ms
        self.__lock_resource = False  # To prevent issuing simultaneous commands to RMCS2303 motor controller. Eg.
        # trying to read encoder value while writing motor enable command
        self.name = self.extract_name_from_port_name(port_name)
        self.__status_rotation_direction = 0
        self.__CW = 1  # clockwise rotation status
        self.__CCW = -1  # counter clockwise rotation status
        self.__IDLE = 0  # no rotation status

        # Functions
        self.__set_lines_per_rotation(rhino.LINES_PER_ROTATION_DEFAULT)
        self.brake()
        self.__go_home()
        self.set_acceleration(rhino.ACCELERATION_DEFAULT)
        self.set_speed(rhino.SPEED_DEFAULT)

    # Private Functions
    # -----------------
    @staticmethod
    def __convert_unsigned32_to_signed32(unsigned32_data):
        # UInt32 range: 0 to 4294967295
        # Int32 range: -2147483648 to 2147483647
        mid_uint32 = 2147483648
        if unsigned32_data is not None:
            signed32_data = int(unsigned32_data - mid_uint32)
            return signed32_data

    @staticmethod
    def __convert_signed32_to_signed16(signed32_data):
        # Int16 range: -32768 to 32767
        signed16_data = signed32_data >> 16
        return signed16_data

    def __read_from_register(self, message_list):
        while True:  # Attempt sending message until the controller is free
            try:
                if not self.__lock_resource:  # Check if controller is in use
                    self.__lock_resource = True
                    data = self.__instrument.read_register(message_list[0], message_list[1], message_list[2])
                    time.sleep(self.__time_delay)
                    self.__lock_resource = False
                    return data
            except KeyboardInterrupt:
                print("Keyboard Interrupt: " + self.name)
            except modbus.ModbusException as e:
                print("ModbusException at " + self.name + ": " + str(e))
            except modbus.serial.SerialException as e:
                print("Modbus Serial Exception at " + self.name + ": " + str(e))
            except modbus.InvalidResponseError as e:
                print("Modbus Invalid Response Exception at " + self.name + ": " + str(e))
            except Exception as e:
                print("Motor Driver Exception at " + self.name + ": " + str(e))
                print(traceback.format_exc())
            time.sleep(self.__time_delay)

    def __read_from_registers(self, message_list):
        while True:  # Attempt sending message until the controller is free
            try:
                if not self.__lock_resource:  # Check if controller is in use
                    self.__lock_resource = True
                    register_size = 16
                    data = self.__instrument.read_registers(message_list[0], message_list[1], message_list[2])
                    lsb = data[0]
                    msb = data[1]
                    combined_data = (msb << register_size) + lsb  # combining two 16 bit values into one 32 bit value
                    time.sleep(self.__time_delay)
                    self.__lock_resource = False
                    return combined_data
                '''
                # combine two registers and create a long integer
                def combine_two_registers(self, reg):
                    if reg[1] > 32767:
                        long_reg = (65535 - reg[1])
                        b = long_reg << 16
                        out = (b + 65535 - reg[0]) * -1
                    else:
                        long_reg = reg[1]
                        b = long_reg << 16
                        out = b + reg[0]
                    return out
                '''
            except KeyboardInterrupt:
                print("Keyboard Interrupt: " + self.name)
            except modbus.ModbusException as e:
                print("ModbusException at " + self.name + ": " + str(e))
            except modbus.serial.SerialException as e:
                print("Modbus Serial Exception at " + self.name + ": " + str(e))
            except modbus.InvalidResponseError as e:
                print("Modbus Invalid Response Exception at " + self.name + ": " + str(e))
            except Exception as e:
                print("Motor Driver Exception at " + self.name + ": " + str(e))
                print(traceback.format_exc())
            time.sleep(self.__time_delay)

    def __write_to_register(self, message_list):
        while True:  # Attempt sending message until the controller is free
            try:
                if not self.__lock_resource:  # Check if controller is in use
                    self.__lock_resource = True
                    self.__instrument.write_register(message_list[0], message_list[1], message_list[2], message_list[3])
                    time.sleep(self.__time_delay)
                    self.__lock_resource = False
                    return
            except KeyboardInterrupt:
                print("Keyboard Interrupt: " + self.name)
            except modbus.ModbusException as e:
                print("ModbusException at " + self.name + ": " + str(e))
            except modbus.serial.SerialException as e:
                print("Modbus Serial Exception at " + self.name + ": " + str(e))
            except modbus.InvalidResponseError as e:
                print("Modbus Invalid Response Exception at " + self.name + ": " + str(e))
            except Exception as e:
                print("Motor Driver Exception at " + self.name + ": " + str(e))
                print(traceback.format_exc())

            time.sleep(self.__time_delay)

    def __go_home(self):
        message = rhino.HOME_POSITION_MESSAGE
        self.__write_to_register(message)

    def __set_lines_per_rotation(self, lines_per_rotation):
        message = rhino.LINES_PER_ROTATION_MESSAGE
        message[rhino.DATA_INDEX] = lines_per_rotation
        self.__write_to_register(message)

    # Public Functions
    # ----------------
    @staticmethod
    def extract_name_from_port_name(port_name):
        chars = port_name.split("/")
        name = chars[len(chars) - 1]
        return name

    @staticmethod
    def convert_rad_per_sec_to_rpm(radians_per_sec):
        # Formula: rpm = rad/sec * 9.549297
        rpm = radians_per_sec * 9.549297
        rpm_scaled = rpm * rhino.GEAR_RATIO
        return rpm_scaled

    @staticmethod
    def convert_rpm_to_rad_per_sec(rpm):
        # Formula: rad/sec = rpm * 0.10472
        radians_per_sec = rpm * 0.10472
        radians_per_sec_scaled = radians_per_sec / rhino.GEAR_RATIO
        return radians_per_sec_scaled

    def set_speed(self, speed):
        speed_rpm = abs(int(self.convert_rad_per_sec_to_rpm(speed)))
        if speed_rpm > rhino.SPEED_MAX:
            speed_rpm = rhino.SPEED_MAX
        if speed_rpm < rhino.SPEED_MIN:
            speed_rpm = rhino.SPEED_MIN
        message = rhino.SPEED_MESSAGE
        message[rhino.DATA_INDEX] = speed_rpm
        self.__write_to_register(message)

    def set_acceleration(self, acceleration):
        if acceleration > rhino.ACCELERATION_MAX:
            acceleration = rhino.ACCELERATION_MAX
        if acceleration < rhino.ACCELERATION_MIN:
            acceleration = rhino.ACCELERATION_MIN
        message = rhino.ACCELERATION_MESSAGE
        message[rhino.DATA_INDEX] = acceleration
        self.__write_to_register(message)

    def turn_motor_cw(self):
        message = rhino.TURN_MOTOR_CW_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__CW

    def turn_motor_ccw(self):
        message = rhino.TURN_MOTOR_CCW_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__CCW

    def stop_rotation_cw(self):
        message = rhino.STOP_MOTOR_CW_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__IDLE

    def stop_rotation_ccw(self):
        message = rhino.STOP_MOTOR_CCW_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__IDLE

    def stop_rotation(self):
        message = rhino.STOP_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__IDLE

    def emergency_stop(self):
        message = rhino.EMERGENCY_STOP_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__IDLE

    def get_position_32bit(self):
        message = rhino.POSITION_FEEDBACK_MESSAGE
        position = self.__read_from_registers(message)
        # position = self.__convert_unsigned32_to_signed32(position)
        return position

    def get_position_16bit(self):
        message = rhino.POSITION_FEEDBACK_MESSAGE
        position = self.__read_from_registers(message)
        position_32bit = self.__convert_unsigned32_to_signed32(position)
        position_16bit = self.__convert_signed32_to_signed16(position_32bit)
        return position_16bit

    def get_position_raw(self):
        message = rhino.POSITION_FEEDBACK_MESSAGE
        position = self.__read_from_registers(message)
        return position

    def get_speed(self):
        message = rhino.SPEED_FEEDBACK_MESSAGE
        speed = self.__read_from_register(message)
        speed = self.__convert_unsigned32_to_signed32(speed)
        return speed

    def brake_cw(self):
        message = rhino.BRAKE_CW_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__IDLE

    def brake_ccw(self):
        message = rhino.BRAKE_CCW_MESSAGE
        self.__write_to_register(message)
        self.__status_rotation_direction = self.__IDLE

    def brake(self):
        if self.__status_rotation_direction == self.__CW:
            self.brake_cw()
            print(self.name + ": Brake CW")
            self.__status_rotation_direction = self.__IDLE
        elif self.__status_rotation_direction == self.__CCW:
            self.brake_ccw()
            print(self.name + ": Brake CCW")
            self.__status_rotation_direction = self.__IDLE
        elif self.__status_rotation_direction == self.__IDLE:
            print(self.name + ": Motor idle")
        else:
            print(self.name + ": Motor Unknown Rotation Status")
