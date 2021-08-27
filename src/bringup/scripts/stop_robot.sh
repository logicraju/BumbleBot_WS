#!/bin/bash
/usr/bin/tmux send-keys -t manager.0 "" C-c
sleep 3
/usr/bin/tmux kill-session -t manager
