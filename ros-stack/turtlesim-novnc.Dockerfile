FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11vnc fluxbox \
    python3-websockify novnc \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Use default RMW (FastDDS) which works better in Docker bridge networks
ENV ROS_DOMAIN_ID=42

ENV DISPLAY=:1
ENV GEOMETRY=800x800x24
ENV VNC_PORT=5901
ENV NOVNC_PORT=6081

COPY turtlesim-novnc-entrypoint.sh /usr/local/bin/turtlesim-novnc-entrypoint.sh
RUN chmod +x /usr/local/bin/turtlesim-novnc-entrypoint.sh

EXPOSE 6081 5901

CMD ["/usr/local/bin/turtlesim-novnc-entrypoint.sh"]

