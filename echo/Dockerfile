# Base image
FROM ros:humble

# Set working directory
WORKDIR /root

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    wget \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rclpy \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-webots-ros2 \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install -U \
    pip \
    argcomplete \
    flake8 \
    pytest \
    setuptools===70.3.0

# Add ROS setup to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export WEBOTS_HOME=/root/.ros"  >> ~/.bashrc
RUN echo "export USER=root"  >> ~/.bashrc




CMD ["ros2", "topic", "echo", "/hello"]
