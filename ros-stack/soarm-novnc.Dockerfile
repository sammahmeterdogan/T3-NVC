FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install Simulator GUI dependencies + MoveIt
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11vnc fluxbox \
    python3-websockify novnc \
    git python3-pip \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Clone the SO-ARM Repo
WORKDIR /root/ws/src
RUN git clone https://github.com/qdeyna/SO-ARM_MoveIt_IsaacSim.git

# Install dependencies
WORKDIR /root/ws
# Simple rosdep (skipping init if already done in base, likely not)
# RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build (Colcon)
RUN . /opt/ros/humble/setup.sh && colcon build

# Environment variables
ENV ROS_DOMAIN_ID=42
ENV DISPLAY=:1
ENV GEOMETRY=1600x900x24
# Ports will be managed by entrypoint/compose but defaults:
ENV VNC_PORT=5901
ENV NOVNC_PORT=6081

# Copy helper scripts
COPY soarm-entrypoint.sh /usr/local/bin/soarm-entrypoint.sh
COPY moveit_wrapper.py /usr/local/bin/moveit_wrapper.py
RUN chmod +x /usr/local/bin/soarm-entrypoint.sh /usr/local/bin/moveit_wrapper.py

EXPOSE 6081 5901

CMD ["/usr/local/bin/soarm-entrypoint.sh"]
