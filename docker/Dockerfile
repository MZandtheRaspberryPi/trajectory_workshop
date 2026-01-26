# this is a dockerfile to get ros2 humble and build for AMD. ARM is not yet supported due to gazebo binaries not being easily available
# for deploying on a robot, gazebo is probably unneeded though.

FROM ubuntu:22.04

ARG UID="1001"

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ca-certificates \
    curl \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*


# Setup ROS Apt sources
RUN curl -L -s -o /tmp/ros2-apt-source.deb https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.jammy_all.deb \
    && echo "1600cb8cc28258a39bffc1736a75bcbf52d1f2db371a4d020c1b187d2a5a083b /tmp/ros2-apt-source.deb" | sha256sum --strict --check \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=humble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update


# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    # ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install sudo before we create user with sudo
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# for unitree_ros2 and DDS IDL generation
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rmw-cyclonedds-cpp \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*


ENV USERNAME=student
# we use uid of 1000, seems to be default in ubuntu but change if different
RUN useradd -U --uid $UID -ms /bin/bash $USERNAME \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && adduser $USERNAME sudo \
    && adduser $USERNAME dialout \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME
# Commands below run as the developer user
USER $USERNAME
# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME


SHELL ["/bin/bash", "-c"]

RUN rosdep fix-permissions && rosdep update --rosdistro $ROS_DISTRO

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN mkdir -p /home/$USERNAME/ros_ws/src

# installing some useful tools
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    nano \
    screen \
    tmux \
    tree \
    xauth \
    iproute2 \
    iputils-ping \
    ros-dev-tools \
    && sudo rm -rf /var/lib/apt/lists/*

COPY dev/ros2_ws/src/unitree_ros2 /home/${USERNAME}/ros_ws/src/unitree_ros2

RUN cd /home/${USERNAME}/ros_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    # && sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list \
    && (sudo rosdep init || true) \
    && rosdep update \
    && sudo apt update \
    && rosdep install --from-paths src --ignore-src -r -y  

RUN cd /home/${USERNAME}/ros_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install \
    && source /home/${USERNAME}/ros_ws/install/setup.bash

RUN cd /home/${USERNAME}/ && sudo apt-get install python3-pip python3-venv -y \
    && git clone https://github.com/MZandtheRaspberryPi/trajectory_workshop.git \
    && cd trajectory_workshop \
    && git checkout 49e52640b15263233d7e86ceaec7fd72d8173aea \
    && python3 -m pip install build \
    && python3 -m build --wheel \
    && sudo python3 -m pip install dist/traj_lib-1.0.0-py3-none-any.whl

# for fastlio
RUN sudo apt install --no-install-recommends libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-ros -y
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && cd ./Livox-SDK2/ && mkdir build && cd build && cmake .. && make -j2 && sudo make install
RUN sudo chmod 777 /usr/local/lib/liblivox_lidar_sdk_static.a