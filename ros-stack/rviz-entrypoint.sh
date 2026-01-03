#!/usr/bin/env bash
set -e

# Start Xvfb virtual display
Xvfb ${DISPLAY} -screen 0 ${GEOMETRY} -ac +render -noreset &
sleep 1

# Start window manager
fluxbox &

# Start VNC server
x11vnc -display ${DISPLAY} -rfbport 5900 -forever -shared -nopw -quiet &

# Start websockify/noVNC (bind to 0.0.0.0 to accept external connections)
websockify --web=/usr/share/novnc/ 0.0.0.0:6080 localhost:5900 &

# Wait for services to start
sleep 2

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Launch RViz2
# Optional: use config file with -d /configs/your_config.rviz
rviz2
