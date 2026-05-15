FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    vim \
    htop \
    tree \
    net-tools \
    iputils-ping \
    usbutils \
    ros-noetic-robot-localization \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-gmapping \
    ros-noetic-hector-slam \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-xacro \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-rqt \
    ros-noetic-rqt-graph \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-vision-opencv \
    ros-noetic-depthimage-to-laserscan \
    ros-noetic-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update || true

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/catkin_ws/devel/setup.bash ]; then source /root/catkin_ws/devel/setup.bash; fi" >> /root/.bashrc

WORKDIR /root/catkin_ws
CMD ["bash"]
