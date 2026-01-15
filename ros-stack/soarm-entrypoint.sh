#!/bin/bash
set -e

# Function to find a free port
find_free_port() {
    local port=$1
    while netstat -atn | grep -q ":$port "; do
        echo "Port $port is in use, trying next..."
        port=$((port + 1))
    done
    echo $port
}

# 1. Port Collision Handling
echo "Checking ports..."
# Ensure net-tools is installed or check via other means. 
# Base ros image is minimal, might miss netstat. 
# Fallback: Python check if netstat missing? 
# Assuming netstat is ok (turtlesim script didn't check? or used fixed).
# I'll rely on env var overrides primarily, but do a check if possible.

# Minimal check:
if command -v netstat >/dev/null; then
    NOVNC_PORT=$(find_free_port ${NOVNC_PORT:-6081})
    VNC_PORT=$(find_free_port ${VNC_PORT:-5901})
else
    echo "netstat not found, using configured ports: $NOVNC_PORT / $VNC_PORT"
fi

echo "----------------------------------------"
echo "SO-ARM SIMULATOR STACK STARTING"
echo "noVNC URL: http://localhost:$NOVNC_PORT/vnc.html"
echo "VNC Port: $VNC_PORT"
echo "----------------------------------------"

# 2. X11 / VNC Setup
Xvfb $DISPLAY -screen 0 $GEOMETRY &
sleep 2
fluxbox &
x11vnc -display $DISPLAY -bg -forever -nopw -quiet -listen localhost -xkb -rfbport $VNC_PORT
/usr/bin/python3 /usr/bin/websockify --web /usr/share/novnc $NOVNC_PORT localhost:$VNC_PORT &

# 3. ROS Setup
source /opt/ros/humble/setup.bash
if [ -f "/root/ws/install/setup.bash" ]; then
    source /root/ws/install/setup.bash
fi

# 4. Launch Rosbridge WebSocket Server (Port 9090 for Frontend)
echo "Starting Rosbridge WebSocket Server on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
sleep 2

# 5. Launch MoveIt Demo (in background)
# Using 'ros2 launch' as identified in repo docs
echo "Launching SO-ARM MoveIt Demo..."
ros2 launch so_arm_moveit_config demo.launch.py &

# 6. Run MoveIt Wrapper (Foreground or Background? Background to keep container alive via tail)
echo "Starting MoveIt Wrapper Node..."
python3 /usr/local/bin/moveit_wrapper.py &

# Keep alive
tail -f /dev/null
