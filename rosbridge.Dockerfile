# ROS2 Humble tabanı
FROM ros:humble-ros-core

# ROSBridge WebSocket sunucusu
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

# Use default RMW (FastDDS) which works better in Docker bridge networks
ENV ROS_DOMAIN_ID=42

EXPOSE 9090

CMD ["bash","-lc","source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090"]
