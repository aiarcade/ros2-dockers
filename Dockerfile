FROM ros:humble

WORKDIR /root

# Install ROS2 and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    wget \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros-base \
    ros-humble-webots-ros2-driver \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-rosbridge-server \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Python tools
RUN pip3 install -U \
    pip \
    argcomplete \
    flake8 \
    pytest \
    setuptools===70.3.0

# Source ROS2 automatically on container launch
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
