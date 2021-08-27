#!/usr/bin/env python3

import time
import traceback
import rhino_params as rhino
import motor_driver
state = 1
driver_left = motor_driver.Controller("/dev/left_wheel", 7)
driver_right = motor_driver.Controller("/dev/right_wheel", 7)
driver_left.set_speed(100)
driver_right.set_speed(100)
encoder_ticks_left_start = 0
encoder_ticks_right_start = 0


def main():
    global state, driver_right, driver_left, encoder_ticks_left_start, encoder_ticks_right_start
    time_threshold = 3
    try:
        while True:
            if state == 1:
                encoder_ticks_left_start = driver_left.get_position()
                encoder_ticks_right_start = driver_right.get_position()
                driver_left.turn_motor_cw()
                driver_right.turn_motor_cw()
                start_time = time.time()
                state = 2

            elif state == 2:
                current_time = time.time()
                time_diff = current_time - start_time
                if time_diff >= time_threshold:
                    state = 3
                speed_left = driver_left.get_speed()
                speed_right = driver_right.get_speed()
                encoder_ticks_left = driver_left.get_position()
                encoder_ticks_right = driver_right.get_position()
                print("Speed Left: " + str(speed_left) + " rpm" + "Encoder Ticks Left: " + str(encoder_ticks_left))
                print("Speed Right: " + str(speed_right) + " rpm" + "Encoder Ticks Right: " + str(encoder_ticks_right))
                '''
                if (abs(encoder_ticks_left_start - encoder_ticks_left) >= rhino.ROTATION_360_ENCODER_COUNT) or \
                        (abs(encoder_ticks_right_start - encoder_ticks_right) >= rhino.ROTATION_360_ENCODER_COUNT):
                    state = 6
                '''
            elif state == 3:
                driver_left.brake_cw()
                driver_right.brake_cw()
                state = 4

            elif state == 4:
                driver_left.turn_motor_ccw()
                driver_right.turn_motor_ccw()
                start_time = time.time()
                state = 5

            elif state == 5:
                current_time = time.time()
                time_diff = current_time - start_time
                if time_diff >= time_threshold:
                    state = 6
                speed_left = driver_left.get_speed()
                speed_right = driver_right.get_speed()
                encoder_ticks_left = driver_left.get_position()
                encoder_ticks_right = driver_right.get_position()
                print("Speed Left: " + str(speed_left) + " rpm" + " Encoder Ticks Left: " + str(encoder_ticks_left))
                print("Speed Right: " + str(speed_right) + " rpm" + " Encoder Ticks Right: " + str(encoder_ticks_right))

            elif state == 6:
                driver_left.brake_ccw()
                driver_right.brake_ccw()
                state = 1
            time.sleep(0.5)
    except Exception as e:
        # driver_left.stop_rotation()
        # driver_right.stop_rotation()
        print("Interrupted ! " + traceback.format_exc())
    finally:
        driver_left.stop_rotation()
        driver_right.stop_rotation()
        print("Final")


if __name__ == "__main__":
    main()
