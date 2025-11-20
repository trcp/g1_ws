# Build image arguments
ARG OS=22.04
ARG CUDA=13.0.0
ARG TARGET_ARCH=arm64
ARG ROS=humble


# =====
# main 
# =====
FROM gai313/ros2:${ROS}.${TARGET_ARCH} AS robot

ARG ROS=humble

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

# build MID-360
USER root
RUN mv livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml &&\
    cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j && sudo make install

# resolve depends
USER $USERNAME
COPY src ./erasers_g1
USER root
RUN . /opt/ros/${ROS}/setup.bash &&\
    apt update &&\
    rosdep install -y -i --from-path . &&\
    rm -rf /var/lib/apt/lists/*

# build workspace
USER $USERNAME
WORKDIR /home/${USERNAME}/colcon_ws
RUN . /opt/ros/${ROS}/setup.bash &&\
    colcon build --symlink-install --packages-up-to erasers_g1 \
    --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=$ROS_DISTRO
