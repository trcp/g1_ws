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
ARG USERNAME
ARG GROUPNAME
ARG UID=1000
ARG GID=1000
ARG PASSWORD

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

# install realsense sdk
USER root
RUN mkdir -p /etc/apt/keyrings &&\
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null &&\
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list &&\
    apt update

# resolve depends
USER $USERNAME
COPY src ./erasers_g1
USER root
RUN . /opt/ros/${ROS}/setup.bash &&\
    apt-get update &&\
    rosdep install -y -i --from-path .\
    --skip-keys pointcloud_to_2dmap \
    --skip-keys pcl_localization_ros2 \
    --skip-keys direct_lidar_inertial_odometry \
    --skip-keys fast_lio &&\
    rm -rf /var/lib/apt/lists/*

# Fix pointcloud2_to_2dmap shared ptr ref
#RUN sed -i '97c\  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();' ./pointcloud_to_2dmap/src/pointcloud_to_2dmap.cpp

# resolve mapeditor depends
RUN apt-get update && apt-get install -y \
    python3-tk \
    python3-pil.imagetk \
    ros-${ROS}-navigation2 ros-${ROS}-nav2-bringup &&\
    rm -rf /var/lib/apt/lists/*

# install python packages
USER $USERNAME
RUN pip install pyserial rustypot transform3d

# COPY amcl2 code
COPY assets/emcl2_node.cpp ./emcl2/src/emcl2_node.cpp

# build workspace
USER $USERNAME
WORKDIR /home/${USERNAME}/colcon_ws
RUN . /opt/ros/${ROS}/setup.bash &&\
    colcon build --symlink-install --packages-up-to erasers_g1 \
    --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=$ROS_DISTRO
