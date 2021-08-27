#!/bin/bash

/usr/bin/tmux -2 new-session -d -s robot_start
/usr/bin/tmux send-keys -t robot_start.0 "roslaunch my_robot_model my_robo_navigation_hardware.launch" ENTER
