FROM ubuntu:22.04

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install necessary packages including ca-certificates
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Setup sources.list for ROS 2 and Gazebo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update the package list and upgrade installed packages
RUN apt-get update && apt-get upgrade -y

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Fortress (which is the version that comes with ROS 2 Humble)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Set the default command to bash
CMD ["bash"]