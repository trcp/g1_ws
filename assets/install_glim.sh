#!/bin/bash
set -euo pipefail

ROS_DISTRO=${1:-}
GLIM_INSTALL_MODE=${2:-auto}
REQUESTED_CUDA_VERSION=${3:-auto}

if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS distribution name is required."
    exit 1
fi

echo "========================================"
echo " Installing GLIM for ROS 2 ${ROS_DISTRO} "
echo " Mode: ${GLIM_INSTALL_MODE}"
echo " CUDA: ${REQUESTED_CUDA_VERSION}"
echo "========================================"

GLIM_WS=/opt/glim_ws
SOURCE_DIR=/tmp/glim_source
GLIM_SETUP_LINE=". ${GLIM_WS}/install/setup.bash"

apt_package_exists() {
    apt-cache show "$1" >/dev/null 2>&1
}

apt_install_common_tools() {
    apt-get update
    apt-get install -y curl gpg sudo
}

setup_glim_ppa() {
    if [ -f /etc/apt/sources.list.d/koide3_ppa.list ]; then
        return 0
    fi

    curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
}

normalize_cuda_version() {
    local version=$1

    echo "$version" | grep -oE '^[0-9]+(\.[0-9]+)?' | head -n 1
}

cuda_package_suffix() {
    local version

    version=$(normalize_cuda_version "$1")
    if [ -z "$version" ]; then
        return 1
    fi
    echo "$version" | tr '.' '-'
}

detect_cuda_version() {
    local cuda_dir=/usr/local/cuda
    local cuda_realpath

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

    if [ "$REQUESTED_CUDA_VERSION" != "auto" ] && [ -n "$REQUESTED_CUDA_VERSION" ]; then
        normalize_cuda_version "$REQUESTED_CUDA_VERSION" && return 0
    fi

    return 1
}

is_jetson() {
    [ -f /etc/nv_tegra_release ] && return 0
    dpkg-query -W nvidia-l4t-core >/dev/null 2>&1 && return 0
    [ -d /usr/lib/aarch64-linux-gnu/tegra ] && return 0
    [ -d /usr/lib/aarch64-linux-gnu/nvidia ] && [ "$(dpkg --print-architecture)" = "arm64" ] && return 0
    return 1
}

configure_cuda_environment() {
    local cuda_version=$1
    local cuda_root=/usr/local/cuda
    local cuda_minor

    cuda_minor=$(normalize_cuda_version "$cuda_version")
    if [ -n "$cuda_minor" ] && [ -d "/usr/local/cuda-${cuda_minor}" ]; then
        cuda_root="/usr/local/cuda-${cuda_minor}"
    elif [ -L /usr/local/cuda ] || [ -d /usr/local/cuda ]; then
        cuda_root=/usr/local/cuda
    fi

    export CUDA_HOME="$cuda_root"
    export CUDAToolkit_ROOT="$cuda_root"
    export PATH="${cuda_root}/bin:${PATH}"
    export LD_LIBRARY_PATH="${cuda_root}/lib64:${cuda_root}/targets/$(uname -m)-linux/lib:${LD_LIBRARY_PATH:-}"
}

ensure_nvcc() {
    local cuda_version=$1
    local suffix

    configure_cuda_environment "$cuda_version"
    if command -v nvcc >/dev/null 2>&1; then
        nvcc --version
        return 0
    fi

    suffix=$(cuda_package_suffix "$cuda_version") || {
        echo "Error: CUDA version is required to install nvcc." >&2
        exit 1
    }

    echo "nvcc was not found. Installing CUDA compiler packages for CUDA ${cuda_version}..."
    apt-get update
    if ! apt-get install -y "cuda-toolkit-${suffix}"; then
        apt-get install -y \
            "cuda-compiler-${suffix}" \
            "cuda-cudart-dev-${suffix}" \
            "cuda-nvcc-${suffix}"
    fi

    configure_cuda_environment "$cuda_version"
    if ! command -v nvcc >/dev/null 2>&1; then
        echo "Error: nvcc is still not available after CUDA compiler installation." >&2
        exit 1
    fi
    nvcc --version
}

install_glim_from_apt() {
    local gtsam_points_package=$1
    local glim_package=$2

    setup_glim_ppa
    apt-get update
    apt-get install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev

    echo "Installing GLIM from apt packages..."
    apt-get install -y "${gtsam_points_package}" "${glim_package}"
}

append_setup_once() {
    local bashrc=$1

    if [ ! -f "$bashrc" ]; then
        return 0
    fi
    if ! grep -Fqx "$GLIM_SETUP_LINE" "$bashrc"; then
        echo "$GLIM_SETUP_LINE" >> "$bashrc"
    fi
}

install_glim_setup_hooks() {
    append_setup_once /etc/bash.bashrc
    append_setup_once /etc/skel/.bashrc

    for home_dir in /home/*; do
        if [ -d "$home_dir" ]; then
            append_setup_once "${home_dir}/.bashrc"
        fi
    done
}

build_glim_from_source() {
    local build_with_cuda=$1
    local cuda_version=${2:-}
    local cmake_cuda_args=()

    if [ "$build_with_cuda" = "ON" ]; then
        if [ -z "$cuda_version" ]; then
            cuda_version=$(detect_cuda_version || true)
        fi
        if [ -z "$cuda_version" ]; then
            echo "Error: CUDA source build was requested, but CUDA was not detected." >&2
            exit 1
        fi
        ensure_nvcc "$cuda_version"
        cmake_cuda_args=(-DCUDAToolkit_ROOT="${CUDAToolkit_ROOT}")
    fi

    echo "Building GLIM from source with CUDA=${build_with_cuda}..."

    apt-get update
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
            -DBUILD_WITH_CUDA="${build_with_cuda}" \
            -DBUILD_WITH_MARCH_NATIVE=OFF \
            "${cmake_cuda_args[@]}"
        cmake --build build -j"$(nproc)"
        cmake --install build
    )

    git clone https://github.com/koide3/glim "${GLIM_WS}/src/glim"
    git clone https://github.com/koide3/glim_ros2 "${GLIM_WS}/src/glim_ros2"
    (
        cd "${GLIM_WS}"
        . "/opt/ros/${ROS_DISTRO}/setup.bash"
        colcon build --merge-install --cmake-args \
            -DBUILD_WITH_CUDA="${build_with_cuda}" \
            -DBUILD_WITH_VIEWER=ON \
            -DBUILD_WITH_MARCH_NATIVE=OFF \
            "${cmake_cuda_args[@]}"
    )

    install_glim_setup_hooks
    rm -rf "${SOURCE_DIR}" "${GLIM_WS}/src" "${GLIM_WS}/build" "${GLIM_WS}/log"
}

choose_auto_mode() {
    local cuda_version

    if is_jetson; then
        echo "cuda-source"
        return 0
    fi

    cuda_version=$(detect_cuda_version || true)
    if [ -z "$cuda_version" ]; then
        echo "cpu"
        return 0
    fi

    echo "cuda-apt"
}

apt_install_common_tools

case "$GLIM_INSTALL_MODE" in
    auto)
        GLIM_INSTALL_MODE=$(choose_auto_mode)
        echo "Resolved GLIM install mode: ${GLIM_INSTALL_MODE}"
        ;;
    cpu|cuda-apt|cuda-source)
        ;;
    *)
        echo "Error: unknown GLIM install mode: ${GLIM_INSTALL_MODE}" >&2
        echo "Valid modes: auto, cpu, cuda-apt, cuda-source" >&2
        exit 2
        ;;
esac

CUDA_VER=$(detect_cuda_version || true)

case "$GLIM_INSTALL_MODE" in
    cpu)
        install_glim_from_apt libgtsam-points-dev "ros-${ROS_DISTRO}-glim-ros"
        ;;
    cuda-apt)
        if [ -z "$CUDA_VER" ]; then
            echo "Error: CUDA apt install was requested, but CUDA was not detected." >&2
            exit 1
        fi

        GTSAM_POINTS_PACKAGE="libgtsam-points-cuda${CUDA_VER}-dev"
        GLIM_ROS_PACKAGE="ros-${ROS_DISTRO}-glim-ros-cuda${CUDA_VER}"

        setup_glim_ppa
        apt-get update
        if apt_package_exists "${GTSAM_POINTS_PACKAGE}" && apt_package_exists "${GLIM_ROS_PACKAGE}"; then
            install_glim_from_apt "${GTSAM_POINTS_PACKAGE}" "${GLIM_ROS_PACKAGE}"
        else
            echo "Error: CUDA ${CUDA_VER} GLIM apt packages are not available." >&2
            echo "Use GLIM_INSTALL_MODE=cuda-source or a GLIM-supported CUDA version." >&2
            exit 1
        fi
        ;;
    cuda-source)
        build_glim_from_source ON "$CUDA_VER"
        ;;
esac

rm -rf /var/lib/apt/lists/*
ldconfig
