# Basic environment values
ARG ARCH=amd64
ARG CTRANSLATE_ARCH=amd64
ARG L4T_VERSION=36.4
ARG TARGET=g1
ARG WORKSPACE_TYPE=base
# voicevox values
ARG VOICEVOX_VERSION=0.17.0
ARG VOICEVOX_ONNXRUNTIME_VERSION=1.23.2
ARG OPEN_JTALK_VERSION=1.11.1
ARG OPEN_JTALK_DICT_VERSION=1.11


# base images
FROM gai313/ros2:humble.jetson.t234.r36.4.runtime AS g1-36.4
FROM gai313/ros2:humble.amd64.cuda12.8.cudnn9.toolkit AS katana-amd64


# ===========
# katana base
# ===========
FROM katana-${ARCH} AS katana-base
# install realsense sdk
USER root
RUN mkdir -p /etc/apt/keyrings &&\
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null &&\
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list &&\
    apt update
# Add user
ARG USERNAME
ARG GROUPNAME
ARG UID=1000
ARG GID=1000
ARG PASSWORD
RUN groupadd -g $GID $GROUPNAME &&\
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME &&\
    echo $USERNAME:$PASSWORD | chpasswd
# Build LiVOX SDK
USER $USERNAME
WORKDIR /home/${USERNAME}
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
USER root
RUN cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j && sudo make install
# Download depends
WORKDIR /home/${USERNAME}/colcon_ws
USER $USERNAME
COPY depends.repos depends.repos
RUN vcs import . < depends.repos
# Optimize pointcloud_to_2dmap for the ROS Humble/PCL toolchain
RUN sed -i 's/boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>()/pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()/' \
    ./thirdparty/pointcloud_to_2dmap/src/pointcloud_to_2dmap.cpp && \
    sed -i 's/${CV_INCLUDE_DIRS}/${OpenCV_INCLUDE_DIRS}/g' \
    ./thirdparty/pointcloud_to_2dmap/CMakeLists.txt
# COPY amcl2 code
COPY assets/emcl2_node.cpp ./thirdparty/emcl2/src/emcl2_node.cpp


# =============
# g1 base image
# =============
FROM g1-${L4T_VERSION} AS g1-base
# Fix OpenCV Version
RUN apt-get update && apt-get install -y --allow-downgrades \
    libopencv-dev=4.5.4+dfsg-9ubuntu4 \
    && apt-mark hold libopencv-dev rsync &&\
    rm -rf /var/lib/apt/lists/*
# install realsense sdk
USER root
RUN mkdir -p /etc/apt/keyrings &&\
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null &&\
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list &&\
    apt update
# Add user
ARG USERNAME
ARG GROUPNAME
ARG UID=1000
ARG GID=1000
ARG PASSWORD
RUN groupadd -g $GID $GROUPNAME &&\
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME &&\
    echo $USERNAME:$PASSWORD | chpasswd
# Build LiVOX SDK
USER $USERNAME
WORKDIR /home/${USERNAME}
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
USER root
RUN cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j && sudo make install
# Download depends
WORKDIR /home/${USERNAME}/colcon_ws
USER $USERNAME
COPY depends.repos depends.repos
RUN vcs import . < depends.repos
# Optimize pointcloud_to_2dmap for the ROS Humble/PCL toolchain
RUN sed -i 's/boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>()/pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()/' \
    ./thirdparty/pointcloud_to_2dmap/src/pointcloud_to_2dmap.cpp && \
    sed -i 's/${CV_INCLUDE_DIRS}/${OpenCV_INCLUDE_DIRS}/g' \
    ./thirdparty/pointcloud_to_2dmap/CMakeLists.txt
# COPY amcl2 code
COPY assets/emcl2_node.cpp ./thirdparty/emcl2/src/emcl2_node.cpp


# ====================
# Model Download
# ====================
FROM gai313/ubuntu:22.04.amd64 AS modeldownloader-amd64
FROM gai313/ubuntu:22.04.arm64 AS modeldownloader-arm64
FROM modeldownloader-${ARCH} AS modeldownloader

# Download openpose weighgt
RUN mkdir -p /tmp/lightweight_openpose && wget -O /tmp/lightweight_openpose/lightweight_openpose.pth https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth
# Download Whisper model
ARG WHISPER_MODEL=faster-whisper-medium
RUN git clone https://huggingface.co/Systran/${WHISPER_MODEL} /tmp/whisper
# Download Depth Model
RUN set -eux; \
    mkdir -p /tmp/hitnet; \
    curl \
        --fail \
        --location \
        --retry 5 \
        --retry-delay 2 \
        --retry-all-errors \
        "https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/142_HITNET/resources.tar.gz" \
        -o /tmp/hitnet_resources.tar.gz; \
    tar \
        -xzf /tmp/hitnet_resources.tar.gz \
        -C /tmp/hitnet; \
    rm -f /tmp/hitnet_resources.tar.gz

# Download YOLO26 Models (Pose & Seg)
RUN set -eux; \
    for MODEL in yolo26l-pose.pt yolo26l-seg.pt; do \
        echo "Downloading ${MODEL}..."; \
        curl \
            --fail \
            --location \
            --show-error \
            --silent \
            --retry 5 \
            --retry-delay 2 \
            --retry-all-errors \
            --connect-timeout 30 \
            "https://huggingface.co/Ultralytics/YOLO26/resolve/main/${MODEL}" \
            --output "/tmp/${MODEL}"; \
        test -s "/tmp/${MODEL}"; \
    done; \
    ls -lh /tmp/yolo26*.pt


# ====================
# Voicevox Wheel Build
# ====================
FROM gai313/ubuntu:22.04.amd64 AS voicevox-amd64
FROM gai313/ubuntu:22.04.arm64 AS voicevox-arm64
FROM voicevox-${ARCH} AS voicevox
ARG VOICEVOX_VERSION

# install build dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        libssl-dev \
        ca-certificates \
        curl \
        git \
        python3-dev \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*
# install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf \
    https://sh.rustup.rs | sh -s -- -y

ENV CARGO_HOME=/root/.cargo
ENV RUSTUP_HOME=/root/.rustup
ENV PATH="/root/.cargo/bin:${PATH}"

# install Python packages
RUN pip install --upgrade pip &&\
    pip install \
    "poetry>=2" \
    "maturin==1.10.2"
# clone voicevox
RUN git clone \
    --branch "${VOICEVOX_VERSION}" \
    --depth 1 \
    https://github.com/VOICEVOX/voicevox_core.git

WORKDIR /voicevox_core
# set version
RUN cargo install cargo-edit \
    --version '^0.11' \
    --locked &&\
    cargo set-version "${VOICEVOX_VERSION}" \
    --exclude voicevox_core_python_api \
    --exclude xtask &&\
    sed -i \
    "s/version = \"0\.0\.0\"/version = \"${VOICEVOX_VERSION}\"/" \
    crates/voicevox_core_python_api/pyproject.toml
# install wheel
RUN rm -rf target/wheels &&\
    cd crates/voicevox_core_python_api &&\
    poetry install --with dev &&\
    cd ../../ &&\
    maturin build \
    --manifest-path crates/voicevox_core_python_api/Cargo.toml \
    --release \
    --locked


# =====================
# Voicevox VVM Download
# =====================
# vvm download environment base images
FROM gai313/ubuntu:22.04.amd64 AS vvm-amd64
FROM gai313/ubuntu:22.04.arm64 AS vvm-arm64
FROM vvm-${ARCH} AS vvm
ARG VOICEVOX_VERSION

# install build dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        git &&\
    rm -rf /var/lib/apt/lists/*
RUN set -eux; \
    mkdir -p /vvm; \
    VVM_BASE_URL="https://github.com/VOICEVOX/voicevox_vvm/releases/download/${VOICEVOX_VERSION}"; \
    \
    for ID in $(seq 0 21); do \
        FILE="${ID}.vvm"; \
        echo "============================================================"; \
        echo "Downloading ${FILE}"; \
        echo "============================================================"; \
        \
        curl \
            --fail \
            --location \
            --show-error \
            --silent \
            --retry 5 \
            --retry-delay 3 \
            --retry-all-errors \
            --connect-timeout 30 \
            "${VVM_BASE_URL}/${FILE}" \
            --output "/vvm/${FILE}"; \
    done
RUN set -eux; \
    VVM_BASE_URL="https://github.com/VOICEVOX/voicevox_vvm/releases/download/0.17.0"; \
    \
    curl \
        --fail \
        --location \
        --show-error \
        --silent \
        --retry 5 \
        --retry-delay 3 \
        --retry-all-errors \
        "${VVM_BASE_URL}/TERMS.txt" \
        --output "/vvm/TERMS.txt"; \
    \
    curl \
        --fail \
        --location \
        --show-error \
        --silent \
        --retry 5 \
        --retry-delay 3 \
        --retry-all-errors \
        "${VVM_BASE_URL}/README.txt"
RUN set -eux; \
    for ID in $(seq 0 21); do \
        FILE="/vvm/${ID}.vvm"; \
        \
        if [ ! -s "${FILE}" ]; then \
            echo "ERROR: VVM file is missing or empty: ${FILE}" >&2; \
            exit 1; \
        fi; \
    done; \
    \
    echo "All VOICEVOX VVM files were downloaded successfully."; \
    ls -lh "/vvm"


# =============================
# Voicevox OnnxRuntime Download
# =============================
# runtime dependency download environment
FROM gai313/ubuntu:22.04.amd64 AS vv_onnxruntime-amd64
FROM gai313/ubuntu:22.04.arm64 AS vv_onnxruntime-arm64
FROM vv_onnxruntime-${ARCH} AS vv_onnxruntime
ARG VOICEVOX_ONNXRUNTIME_VERSION
ARG OPEN_JTALK_VERSION
ARG OPEN_JTALK_DICT_VERSION

# install download / extract dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        tar \
        gzip && \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    mkdir -p /onnxruntime; \
    \
    case "$(dpkg --print-architecture)" in \
        amd64) \
            ORT_ARCH="x64"; \
            ;; \
        arm64) \
            ORT_ARCH="arm64"; \
            ;; \
        *) \
            echo "ERROR: Unsupported architecture for VOICEVOX ONNX Runtime: $(dpkg --print-architecture)" >&2; \
            exit 1; \
            ;; \
    esac; \
    \
    ORT_FILE="voicevox_onnxruntime-linux-${ORT_ARCH}-${VOICEVOX_ONNXRUNTIME_VERSION}.tgz"; \
    ORT_URL="https://github.com/VOICEVOX/onnxruntime-builder/releases/download/voicevox_onnxruntime-${VOICEVOX_ONNXRUNTIME_VERSION}/${ORT_FILE}"; \
    \
    curl \
        --fail \
        --location \
        --show-error \
        --silent \
        --retry 5 \
        --retry-delay 3 \
        --retry-all-errors \
        --connect-timeout 30 \
        "${ORT_URL}" \
        --output "/tmp/${ORT_FILE}"; \
    \
    tar \
        --extract \
        --gzip \
        --file "/tmp/${ORT_FILE}" \
        --directory /onnxruntime \
        --strip-components=1; \
    \
    rm -f "/tmp/${ORT_FILE}"; \
    \
    test -s \
        "/onnxruntime/lib/libvoicevox_onnxruntime.so.${VOICEVOX_ONNXRUNTIME_VERSION}"; \
    \
    echo "VOICEVOX ONNX Runtime successfully installed."; \
    find /onnxruntime -maxdepth 2 -type f -o -type l
RUN set -eux; \
    mkdir -p /dict; \
    \
    DICT_FILE="open_jtalk_dic_utf_8-${OPEN_JTALK_DICT_VERSION}.tar.gz"; \
    DICT_URL="https://github.com/r9y9/open_jtalk/releases/download/v${OPEN_JTALK_VERSION}/${DICT_FILE}"; \
    \
    curl \
        --fail \
        --location \
        --show-error \
        --silent \
        --retry 5 \
        --retry-delay 3 \
        --retry-all-errors \
        --connect-timeout 30 \
        "${DICT_URL}" \
        --output "/tmp/${DICT_FILE}"; \
    \
    tar \
        --extract \
        --gzip \
        --file "/tmp/${DICT_FILE}" \
        --directory /dict; \
    \
    rm -f "/tmp/${DICT_FILE}"; \
    \
    test -s \
        "/dict/open_jtalk_dic_utf_8-${OPEN_JTALK_DICT_VERSION}/char.bin"; \
    test -s \
        "/dict/open_jtalk_dic_utf_8-${OPEN_JTALK_DICT_VERSION}/sys.dic"; \
    \
    echo "Open JTalk dictionary successfully installed."; \
    ls -lh \
        "/dict/open_jtalk_dic_utf_8-${OPEN_JTALK_DICT_VERSION}/"



# =================
# CTranslate2 Build
# =================
FROM gai313/ubuntu:22.04.amd64.cuda12.8.cudnn9.toolkit AS ctranslate2-amd64
FROM gai313/ubuntu:22.04.arm64 AS ctranslate2-arm64
FROM g1-base AS ctranslate2-jetson

FROM ctranslate2-${CTRANSLATE_ARCH} AS ctranslate2

# install build tools
USER root
RUN apt-get update &&\
    apt-get install -y \
        cmake \
        libprotobuf-dev protobuf-compiler \
        libcurl4-openssl-dev \
        libssl-dev \
        zlib1g-dev \
        python3-dev python3-pip python3-setuptools \
        build-essential \
    &&\
    rm -rf /var/lib/apt/lists/*
RUN git clone --recursive https://github.com/OpenNMT/CTranslate2.git /CTranslate2
WORKDIR /CTranslate2
RUN set -eux; \
    CMAKE_FLAGS="-DWITH_MKL=OFF -DOPENMP_RUNTIME=NONE"; \
    if [ -d /usr/local/cuda ]; then \
        echo "CUDA directory detected; forcing CUDA and cuDNN support for CTranslate2."; \
        CMAKE_FLAGS="${CMAKE_FLAGS} -DWITH_CUDA=ON -DWITH_CUDNN=ON"; \
    else \
        echo "CUDA not detected; building CPU-only CTranslate2."; \
        CMAKE_FLAGS="${CMAKE_FLAGS} -DWITH_CUDA=OFF -DWITH_CUDNN=OFF"; \
    fi; \
    cmake -Bbuild_folder ${CMAKE_FLAGS}; \
    cmake --build build_folder --parallel $(nproc)
RUN cd build_folder && make install
RUN pip3 install -r python/install_requirements.txt &&\
    cd python && CTRANSLATE2_ROOT=/usr/local python3 setup.py bdist_wheel


# ==========
# GLIM build
# ==========
FROM gai313/ros2:humble.amd64.cuda12.8.cudnn9.toolkit AS glim-amd64
FROM gai313/ubuntu:22.04.arm64 AS glim-arm64
FROM g1-base AS glim-jetson

FROM glim-${CTRANSLATE_ARCH} AS glim
ARG CUDA_ARCHITECTURES=87
USER root

# Install build dependencies for GLIM and submodules
RUN apt-get update && apt-get install -y \
    --no-install-recommends \
    --allow-downgrades \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libfmt-dev \
    libglfw3-dev \
    libglm-dev \
    libjpeg-dev \
    libmetis-dev \
    libomp-dev \
    libpng-dev \
    libspdlog-dev \
    libopencv-dev=4.5.4+dfsg-9ubuntu4 &&\
    apt-mark hold libopencv-dev rsync &&\
    rm -rf /var/lib/apt/lists/*

ENV CUDA_HOME=/usr/local/cuda
ENV CUDAToolkit_ROOT=/usr/local/cuda
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}

WORKDIR /tmp/glim_build
# GTSAM
RUN git clone --branch 4.3a0 --depth 1 https://github.com/borglab/gtsam.git gtsam && \
    cmake -S gtsam -B gtsam/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    cmake --build gtsam/build -j$(nproc) && \
    cmake --install gtsam/build && \
    rm -rf gtsam
# iridescence
RUN git clone --recursive --depth 1 https://github.com/koide3/iridescence.git && \
    cmake -S iridescence -B iridescence/build \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build iridescence/build -j$(nproc) && \
    cmake --install iridescence/build && \
    rm -rf iridescence
# gtsam_points
RUN git clone --depth 1 https://github.com/koide3/gtsam_points.git && \
    cmake -S gtsam_points -B gtsam_points/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_WITH_CUDA=ON \
        -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES} \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DCUDAToolkit_ROOT=/usr/local/cuda && \
    cmake --build gtsam_points/build -j$(nproc) && \
    cmake --install gtsam_points/build && \
    rm -rf gtsam_points
# glim standalone core
RUN . /opt/ros/humble/setup.bash && \
    git clone --depth 1 https://github.com/koide3/glim.git && \
    cmake -S glim -B glim/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_WITH_CUDA=ON \
        -DBUILD_WITH_VIEWER=ON \
        -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES} \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DCUDAToolkit_ROOT=/usr/local/cuda && \
    cmake --build glim/build -j$(nproc) && \
    cmake --install glim/build && \
    rm -rf glim /tmp/glim_build


# =========================
# eR@sers G1 Workspace base
# =========================
FROM ${TARGET}-base AS base

# resolve depends
USER $USERNAME
COPY src ./src
USER root
RUN . /opt/ros/humble/setup.bash &&\
    apt-get update &&\
    rosdep update --rosdistro=$ROS_DISTRO &&\
    rosdep install -y -i --from-path . \
        --skip-keys pointcloud_to_2dmap \
        --skip-keys glim \
        --skip-keys glim_ros \
        --skip-keys rviz2 \
        --skip-keys joint_state_publisher_gui \
        --skip-keys unitree_sdk2 \
        --skip-keys unitree_ros2 &&\
    rm -rf /var/lib/apt/lists/*


# ============================
# eR@sers G1 Workspace Desktop
# ============================
FROM ${TARGET}-base AS desktop
# resolve depends
USER $USERNAME
COPY src ./src
USER root
RUN . /opt/ros/humble/setup.bash &&\
    apt-get update &&\
    rosdep update --rosdistro=$ROS_DISTRO &&\
    rosdep install -y -i --from-path . \
        --skip-keys pointcloud_to_2dmap \
        --skip-keys glim \
        --skip-keys glim_ros \
        --skip-keys unitree_sdk2 \
        --skip-keys unitree_ros2 &&\
    rm -rf /var/lib/apt/lists/*
USER $USERNAME


# ==========================
# Merge eR@sers G1 Workspace
# ==========================
FROM ${WORKSPACE_TYPE} AS g1_ws
# install Python requirements
USER $USERNAME
RUN mkdir -p voicevox/vvm voicevox/onnxruntime voicevox/dict
# COPY materials
COPY ./requirements.txt ./requirements.txt 
COPY --from=voicevox /voicevox_core/target/wheels/*.whl ./voicevox
COPY --from=vvm /vvm/ ./voicevox/vvm/
COPY --from=vv_onnxruntime /onnxruntime/ ./voicevox/onnxruntime/
COPY --from=vv_onnxruntime /dict/ ./voicevox/dict/
COPY --from=ctranslate2 /usr/local/lib/libctranslate2* /usr/local/lib/
COPY --from=ctranslate2 /usr/local/include/ctranslate2/ /usr/local/include/ctranslate2/
COPY --from=ctranslate2 /CTranslate2/python /tmp/ctranslate2/python
COPY --from=modeldownloader /tmp/hitnet /tmp/hitnet
COPY --from=modeldownloader /tmp/whisper /tmp/whisper
COPY --from=modeldownloader /tmp/lightweight_openpose/lightweight_openpose.pth /tmp/lightweight_openpose/lightweight_openpose.pth
COPY --from=modeldownloader /tmp/yolo26l-pose.pt /tmp/yolo26l-pose.pt
COPY --from=modeldownloader /tmp/yolo26l-seg.pt /tmp/yolo26l-seg.pt
COPY ./onnxruntime/build/Linux/Release/libonnxruntime*.so* /usr/local/lib/
COPY ./onnxruntime/include/onnxruntime /usr/local/include/onnxruntime
COPY ./onnxruntime/build /tmp/onnxruntime
# GLIM artifacts (GTSAM, iridescence, gtsam_points, GLIM core & plugins)
COPY --from=glim /usr/local/lib/ /usr/local/lib/
COPY --from=glim /usr/local/include/ /usr/local/include/
COPY --from=glim /usr/local/share/glim /usr/local/share/glim
# Install onnxruntime & update ldconfig
USER root
RUN ldconfig
# Install requirements Python packages
USER $USERNAME
RUN pip install voicevox/*.whl &&\
    pip install -r requirements.txt &&\
    pip uninstall -y onnxruntime && \
    pip install --no-deps /tmp/onnxruntime/Linux/Release/dist/*.whl && \
    pip install --force-reinstall --no-deps /tmp/ctranslate2/python/dist/*.whl
CMD ["/bin/bash"]
