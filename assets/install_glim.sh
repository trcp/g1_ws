#!/bin/bash
set -e

ROS_DISTRO=$1
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS distribution name is required."
    exit 1
fi
REQUESTED_CUDA_VER=${2:-auto}

echo "========================================"
echo " Installing GLIM for ROS 2 ${ROS_DISTRO} "
echo "========================================"

apt-get update
apt-get install -y curl gpg sudo
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

apt-get update
apt-get install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev

if [ "$REQUESTED_CUDA_VER" != "auto" ]; then
    CUDA_VER="$REQUESTED_CUDA_VER"
    echo "Using requested CUDA Version: ${CUDA_VER}"
elif command -v nvcc &> /dev/null; then
    CUDA_VER=$(nvcc --version | grep "release" | sed -E 's/.*release ([0-9]+\.[0-9]+).*/\1/')
    echo "Detected CUDA Version: ${CUDA_VER}"
else
    CUDA_VER=""
    echo "No CUDA detected. Installing CPU version."
fi

case "$CUDA_VER" in
    "12.2"|"12.6"|"13.1")
        echo "Installing GLIM with CUDA ${CUDA_VER} support..."
        apt-get install -y libgtsam-points-cuda${CUDA_VER}-dev ros-${ROS_DISTRO}-glim-ros-cuda${CUDA_VER}
        ;;
    "")
        apt-get install -y libgtsam-points-dev ros-${ROS_DISTRO}-glim-ros
        ;;
    *)
        echo "Warning: Official PPA does not support CUDA ${CUDA_VER}. Falling back to CPU version."
        apt-get install -y libgtsam-points-dev ros-${ROS_DISTRO}-glim-ros
        ;;
esac

rm -rf /var/lib/apt/lists/*
ldconfig
