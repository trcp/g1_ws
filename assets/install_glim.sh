#!/bin/bash
set -e

ROS_DISTRO=$1
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS distribution name is required."
    exit 1
fi

echo "========================================"
echo " Installing GLIM for ROS 2 ${ROS_DISTRO} "
echo "========================================"

GLIM_WS=/opt/glim_ws
SOURCE_DIR=/tmp/glim_source

apt_package_exists() {
    apt-cache show "$1" >/dev/null 2>&1
}

detect_cuda_version() {
    local cuda_dir=/usr/local/cuda
    local cuda_realpath

    if [ ! -d "${cuda_dir}" ]; then
        return 1
    fi

    if [ -f "${cuda_dir}/version.json" ]; then
        grep -oE '"version"[[:space:]]*:[[:space:]]*"[0-9]+\.[0-9]+' "${cuda_dir}/version.json" \
            | head -n 1 \
            | sed -E 's/.*"([0-9]+\.[0-9]+)$/\1/' && return 0
    fi

    if [ -f "${cuda_dir}/version.txt" ]; then
        grep -oE '[0-9]+\.[0-9]+' "${cuda_dir}/version.txt" | head -n 1 && return 0
    fi

    cuda_realpath=$(readlink -f "${cuda_dir}" 2>/dev/null || true)
    if [ -n "${cuda_realpath}" ]; then
        echo "${cuda_realpath}" | grep -oE 'cuda-?[0-9]+\.[0-9]+' | tail -n 1 | grep -oE '[0-9]+\.[0-9]+' && return 0
    fi

    return 1
}

install_glim_from_apt() {
    local gtsam_points_package=$1
    local glim_package=$2

    echo "Installing GLIM from apt packages..."
    apt-get install -y "${gtsam_points_package}" "${glim_package}"
}

build_glim_from_source() {
    echo "Building GLIM from source with CUDA support..."

    apt-get install -y \
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
        libspdlog-dev

    rm -rf "${SOURCE_DIR}" "${GLIM_WS}"
    mkdir -p "${SOURCE_DIR}" "${GLIM_WS}/src"

    git clone https://github.com/borglab/gtsam "${SOURCE_DIR}/gtsam"
    (
        cd "${SOURCE_DIR}/gtsam"
        git checkout 4.3a0
        cmake -S . -B build \
            -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
            -DGTSAM_BUILD_TESTS=OFF \
            -DGTSAM_WITH_TBB=OFF \
            -DGTSAM_USE_SYSTEM_EIGEN=ON \
            -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
        cmake --build build -j"$(nproc)"
        cmake --install build
    )

    git clone --recursive https://github.com/koide3/iridescence "${SOURCE_DIR}/iridescence"
    (
        cd "${SOURCE_DIR}/iridescence"
        cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
        cmake --build build -j"$(nproc)"
        cmake --install build
    )

    git clone https://github.com/koide3/gtsam_points "${SOURCE_DIR}/gtsam_points"
    (
        cd "${SOURCE_DIR}/gtsam_points"
        cmake -S . -B build \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_WITH_CUDA=ON \
            -DBUILD_WITH_MARCH_NATIVE=OFF
        cmake --build build -j"$(nproc)"
        cmake --install build
    )

    git clone https://github.com/koide3/glim "${GLIM_WS}/src/glim"
    git clone https://github.com/koide3/glim_ros2 "${GLIM_WS}/src/glim_ros2"
    (
        cd "${GLIM_WS}"
        . "/opt/ros/${ROS_DISTRO}/setup.bash"
        colcon build --merge-install --cmake-args \
            -DBUILD_WITH_CUDA=ON \
            -DBUILD_WITH_VIEWER=ON \
            -DBUILD_WITH_MARCH_NATIVE=OFF
    )

    if ! grep -q ". ${GLIM_WS}/install/setup.bash" /etc/skel/.bashrc; then
        echo ". ${GLIM_WS}/install/setup.bash" >> /etc/skel/.bashrc
    fi

    rm -rf "${SOURCE_DIR}" "${GLIM_WS}/src" "${GLIM_WS}/build" "${GLIM_WS}/log"
}

apt-get update
apt-get install -y curl gpg sudo
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

apt-get update
apt-get install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev

if [ -d /usr/local/cuda ]; then
    CUDA_VER=$(detect_cuda_version || true)

    if [ -n "${CUDA_VER}" ]; then
        echo "Detected CUDA Version: ${CUDA_VER}"

        GTSAM_POINTS_PACKAGE="libgtsam-points-cuda${CUDA_VER}-dev"
        GLIM_ROS_PACKAGE="ros-${ROS_DISTRO}-glim-ros-cuda${CUDA_VER}"

        if apt_package_exists "${GTSAM_POINTS_PACKAGE}" && apt_package_exists "${GLIM_ROS_PACKAGE}"; then
            echo "Installing GLIM with CUDA ${CUDA_VER} support from apt..."
            install_glim_from_apt "${GTSAM_POINTS_PACKAGE}" "${GLIM_ROS_PACKAGE}"
        else
            echo "CUDA ${CUDA_VER} GLIM apt packages are not available."
            build_glim_from_source
        fi
    else
        echo "CUDA directory exists but version could not be detected. Building GLIM from source with CUDA support."
        build_glim_from_source
    fi
else
    echo "No CUDA detected. Installing CPU version."
    install_glim_from_apt libgtsam-points-dev ros-${ROS_DISTRO}-glim-ros
fi

rm -rf /var/lib/apt/lists/*
ldconfig
