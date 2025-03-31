#!/bin/bash
set -e

# Source ROS environment
# source /root/project_files/root/smartenv/devel/setup.bash

# Set up fnm environment
eval "$(/root/.fnm/fnm env --use-on-cd --shell bash)"
/root/.fnm/fnm use 20

# Start Xvfb on DISPLAY=:100
Xvfb :100 -screen 0 1024x768x16 &

# Give Xvfb time to start
sleep 2

# Start the VNC server
x11vnc -display :100 -nopw -forever -shared &


# Keep the script running
tail -f /dev/null
