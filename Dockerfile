FROM osrf/ros:humble-desktop
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install core system utilities, Gazebo, Nav2 suite, and your exact mapping tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-rmw-cyclonedds-cpp \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# 2. Install exact Python packages extracted from your environment profile
RUN pip3 install --no-cache-dir \
    numpy==1.26.4 \
    opencv-python==4.12.0.88 \
    torch \
    torchvision \
    ultralytics==8.4.51

# 3. Set up simulation workspace directories
ENV WORKSPACE=/root/ros2_ws
WORKDIR $WORKSPACE

# 4. Copy your local package source code into the container filesystem
COPY ./src /$WORKSPACE/src

# 5. Run rosdep to resolve background dependencies defined in package.xml
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# 6. Lock environment default settings
ENV TURTLEBOT3_MODEL=waffle
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 7. Compile the ROS 2 packages
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# 8. Automate setup sourcing for every terminal session
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
