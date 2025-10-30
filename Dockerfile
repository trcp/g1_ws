# Build image arguments
ARG OS=22.04
ARG CUDA=13.0.0
ARG TARGET_ARCH=arm64

# Target source images
FROM ubuntu:${OS} AS base-amd64
FROM arm64v8/ubuntu:${OS} AS base-arm64
FROM nvidia/cuda:${CUDA}-runtime-ubuntu${OS} AS base-cuda

# =======================================================
LABEL com.example.vendor="Gai Nakatogawa" \
      version="2.0"
# =======================================================

# ROS2 ==================================================
FROM base-${TARGET_ARCH} AS ros2

# Build options
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ENV LANG=ja_JP.UTF-8
ENV TZ=asia/Tokyo

# time zone setup
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo locales software-properties-common tzdata \
    x11-utils x11-apps xauth \
    nano \
    terminator \
    python3-pip curl &&\
    locale-gen ja_JP ja_JP.UTF-8 && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 &&\
    add-apt-repository universe &&\
    rm -rf /var/lib/apt/lists/*

# ROS2 install
ARG ROS=humble
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') &&\
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" &&\
    apt-get install -y /tmp/ros2-apt-source.deb &&\
    apt-get update && apt-get upgrade -y &&\
    apt-get install -y ros-${ROS}-ros-base ros-${ROS}-rmw-fastrtps-cpp ros-${ROS}-rmw-cyclonedds-cpp ros-dev-tools &&\
    rosdep init && rosdep update &&\
    rm -rf /var/lib/apt/lists/*

# general settings
RUN echo ". /opt/ros/${ROS}/setup.bash" >> /etc/skel/.bashrc &&\
    echo ". ~/colcon_ws/install/setup.bash" >> /etc/skel/.bashrc &&\
    echo "export PS1='\[\033[47;30m\]\$ROS_DISTRO\[\033[0m\]:\[\033[32m\]\u:\[\033[36m\]\w \[\033[37m\]\$ '" >> /etc/skel/.bashrc &&\
    mkdir /etc/skel/colcon_ws

# terminator & nano
COPY assets/terminator_config /etc/skel/.config/terminator/config
COPY assets/nanorc /etc/skel/.nanorc
# =======================================================

# main ================================================
FROM ros2 AS robot
# build args
ARG USERNAME=unitree
ARG GROUPNAME=unitree
ARG UID=1000
ARG GID=1000
ARG PASSWORD=123


# Add user
RUN groupadd -g $GID $GROUPNAME &&\
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME &&\
    echo $USERNAME:$PASSWORD | chpasswd
    #echo "$USERNAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add workspace
USER $USERNAME
WORKDIR /home/${USERNAME}/colcon_ws/src
COPY depends.repos depends.repos
RUN . /opt/ros/${ROS}/setup.bash &&\
    vcs import . < depends.repos

COPY src ./erasers_g1
USER root
RUN . /opt/ros/${ROS}/setup.bash &&\
    apt update &&\
    rosdep install -y -i --from-path . &&\
    rm -rf /var/lib/apt/lists/*

# build MID-360
RUN mv livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml &&\
    cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j && sudo make install

USER $USERNAME
WORKDIR /home/${USERNAME}/colcon_ws
RUN . /opt/ros/${ROS}/setup.bash &&\
    colcon build --symlink-install --packages-up-to erasers_g1 \
    --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=$ROS_DISTRO
# =======================================================
