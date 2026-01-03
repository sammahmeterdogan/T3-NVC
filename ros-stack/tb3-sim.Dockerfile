FROM ros:humble-ros-base

# CycloneDDS + Gazebo + TB3 deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    git python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs ros-humble-xacro \
    ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-turtlebot3-msgs ros-humble-turtlebot3-description \
 && rm -rf /var/lib/apt/lists/*

# TB3 simulations (Humble branch) - kaynak derleme
ENV WS=/opt/tb3_ws
RUN mkdir -p $WS/src && cd $WS/src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    . /opt/ros/humble/setup.sh && cd $WS && colcon build --merge-install

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42
