#!/bin/bash

# Change to the directory where you want to serve files (replace "<your_package_name>/www" with the appropriate path)
cd "/home/rajesh/ROS_Workspaces/ROS1/BumbleBot_WS/src"

# Start the HTTP server on port 8000
python3 -m http.server 8000