#!/usr/bin/env bash
set -eo pipefail

VNC_PORT=${VNC_PORT:-5901}
NOVNC_PORT=${NOVNC_PORT:-6081}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

echo "[turtlesim-novnc] DISPLAY=${DISPLAY:-:1} GEOMETRY=${GEOMETRY:-800x800x24}"
echo "[turtlesim-novnc] VNC_PORT=${VNC_PORT} NOVNC_PORT=${NOVNC_PORT}"
echo "[turtlesim-novnc] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[turtlesim-novnc] Starting services..."

# Source ROS environment (handle unbound variables gracefully)
set +u
source /opt/ros/humble/setup.bash
set -u

# Start virtual display
Xvfb ${DISPLAY} -screen 0 ${GEOMETRY} -ac +render -noreset &
sleep 1

# Lightweight window manager
fluxbox &

# Launch turtlesim node (background)
ros2 run turtlesim turtlesim_node &
TURTLE_PID=$!

# Start VNC server
x11vnc -display ${DISPLAY} -rfbport ${VNC_PORT} -forever -shared -nopw -quiet &

# Start noVNC / websockify on 0.0.0.0
websockify --web=/usr/share/novnc/ 0.0.0.0:${NOVNC_PORT} localhost:${VNC_PORT} &

echo "[turtlesim-novnc] noVNC available at http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale"

# Wait for /turtle1/cmd_vel to appear (timeout ~15s) without exiting early on transient errors
tries=30
while true; do
  if ros2 topic list >/tmp/ros_topics 2>&1; then
    if grep -q "/turtle1/cmd_vel" /tmp/ros_topics; then
      echo "[turtlesim-novnc] /turtle1/cmd_vel detected"
      break
    fi
  else
    cat /tmp/ros_topics 2>/dev/null || true
  fi
  tries=$((tries-1))
  if [ $tries -le 0 ]; then
    echo "[turtlesim-novnc] WARNING: /turtle1/cmd_vel not found after 15s"
    echo "[turtlesim-novnc] Nodes:"
    ros2 node list || true
    echo "[turtlesim-novnc] Topics:"
    ros2 topic list || true
    echo "[turtlesim-novnc] Continuing anyway (turtlesim may be starting)..."
    break
  fi
  sleep 0.5
done

# Keep container alive; wait for any background process
# If turtlesim dies, container will exit
wait

