# Build image arguments
ARG OS=22.04
ARG CUDA=13.0.0
ARG L4T_VERSION=r36.4.0
ARG TARGET_ARCH=amd64
ARG ROS=humble


# ===============
# base image list 
# ===============
FROM gai313/ros2:${ROS}.amd64 AS ros2-amd64
FROM gai313/ros2:${ROS}.arm64 AS ros2-arm64
FROM gai313/ros2:${ROS}.cuda.${CUDA} AS ros2-cuda
FROM gai313/ros2:humble.jetpack.${L4T_VERSION} AS ros2-jetpack


# ==========
# base image
# ==========
FROM ros2-${TARGET_ARCH} AS main

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
RUN pip install --index-url https://pypi.org/simple pyserial rustypot transform3d faster-whisper "numpy==1.22.4"

# COPY amcl2 code
COPY assets/emcl2_node.cpp ./emcl2/src/emcl2_node.cpp

# build workspace
USER $USERNAME
WORKDIR /home/${USERNAME}/colcon_ws
RUN . /opt/ros/${ROS}/setup.bash &&\
    colcon build --symlink-install --packages-up-to erasers_g1 \
    --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=$ROS_DISTRO


# =====
# GR00T
# =====
FROM main AS gr00t

USER root
RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev \
    libzmq3-dev \
    git-lfs \
    && rm -rf /var/lib/apt/lists/*

# Download GR00T
USER $USERNAME
WORKDIR /home/${USERNAME}
RUN git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git &&\
    cd GR00T-WholeBodyControl &&\
    git lfs pull

# Download Tensor RT
RUN wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.7.0/tars/TensorRT-10.7.0.23.l4t.aarch64-gnu.cuda-12.6.tar.gz -O TensorRT.tar.gz
RUN tar -xzf TensorRT.tar.gz && ls &&\
    mv TensorRT-10.7.0.23 TensorRT && \
    rm TensorRT.tar.gz

# GR00T build dependencies
USER root
RUN apt-get update && apt-get install -y \
    build-essential \
    clang \
    cmake \
    git \
    git-lfs \
    pkg-config \
    patchelf \
    zlib1g-dev \
    curl \
    wget \
    libyaml-cpp-dev \
    libeigen3-dev \
    libmsgpack-dev \
    libzmq3-dev \
    nlohmann-json3-dev \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Install cppzmq headers (header-only) manually since not in 22.04 repo
RUN wget -q https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq.hpp -P /usr/local/include && \
    wget -q https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq_addon.hpp -P /usr/local/include

# ONNX Runtime (Required for GR00T)
USER root
RUN ONNX_VERSION="1.16.3" && \
    ONNX_ARCH=$(uname -m) && \
    if [ "${ONNX_ARCH}" = "x86_64" ]; then ONNX_ARCH_NAME="x64"; else ONNX_ARCH_NAME="aarch64"; fi && \
    wget -q "https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-${ONNX_ARCH_NAME}-${ONNX_VERSION}.tgz" -O /tmp/onnxruntime.tgz && \
    mkdir -p /opt/onnxruntime && \
    tar xzf /tmp/onnxruntime.tgz -C /opt/onnxruntime --strip-components=1 && \
    rm /tmp/onnxruntime.tgz

# just (Direct binary download from GitHub)
USER root
RUN JUST_VERSION="1.43.0" && \
    JUST_ARCH=$(uname -m) && \
    case ${JUST_ARCH} in \
        x86_64) JUST_ARCH_NAME="x86_64-unknown-linux-musl" ;; \
	    aarch64|arm64) JUST_ARCH_NAME="aarch64-unknown-linux-musl" ;; \
        *) echo "Unsupported architecture: ${JUST_ARCH}"; exit 1 ;; \
    esac && \
    wget -qO- "https://github.com/casey/just/releases/download/${JUST_VERSION}/just-${JUST_VERSION}-${JUST_ARCH_NAME}.tar.gz" | tar xz -C /usr/local/bin just && \
    chmod +x /usr/local/bin/just

# build settings
ENV TensorRT_ROOT=/home/${USERNAME}/TensorRT
ENV LD_LIBRARY_PATH=${TensorRT_ROOT}/lib:/usr/lib/aarch64-linux-gnu/nvidia:${LD_LIBRARY_PATH}
ENV CMAKE_PREFIX_PATH=${TensorRT_ROOT}:/opt/onnxruntime:${CMAKE_PREFIX_PATH}

# deploy script
COPY assets/deploy_gr00t.sh /home/${USERNAME}/deploy_gr00t.sh
USER root
RUN chmod +x /home/${USERNAME}/deploy_gr00t.sh
USER $USERNAME

RUN pip install --index-url https://pypi.org/simple huggingface_hub && \
    python3 ./GR00T-WholeBodyControl/download_from_hf.py
