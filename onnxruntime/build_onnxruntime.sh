#!/bin/bash
set -e

# 1. CPU アーキテクチャの自動判別
ARCH="$(uname -m)"
case "$ARCH" in
  aarch64|arm64)
    DEFAULT_CUDNN_PATH="/usr/lib/aarch64-linux-gnu"
    DEFAULT_CUDA_ARCH="87"        # Jetson Orin (T234)
    ;;
  x86_64|amd64)
    DEFAULT_CUDNN_PATH="/usr/lib/x86_64-linux-gnu"
    DEFAULT_CUDA_ARCH="86"        # RTX 30系 (Ampere)。RTX 40系なら 89、複数は "80;86;89"
    ;;
  *)
    DEFAULT_CUDNN_PATH="/usr/lib"
    DEFAULT_CUDA_ARCH="native"
    ;;
esac

# 2. パス設定（環境変数が未定義の場合のみデフォルトを適用）
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
CUDNN_HOME="${CUDNN_HOME:-$DEFAULT_CUDNN_PATH}"
TARGET_CUDA_ARCH="${TARGET_CUDA_ARCH:-$DEFAULT_CUDA_ARCH}"

# 3. CUDA & cuDNN の存在確認
HAS_CUDA=false
if [ -x "${CUDA_HOME}/bin/nvcc" ] || command -v nvcc &> /dev/null; then
  if [ -d "${CUDNN_HOME}" ] || [ -d "${CUDA_HOME}" ]; then
    HAS_CUDA=true
  fi
fi

# 4. 利用可能な GPU の確認
HAS_GPU=false
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
  HAS_GPU=true
elif [ -e /dev/nvhost-gpu ] || [ -e /dev/nvhost-ctrl-gpu ] || [ -e /dev/nvidia0 ]; then
  HAS_GPU=true
fi

# 5. 共通ビルドオプション
BUILD_ARGS=(
  --allow_running_as_root
  --config Release
  --update
  --build
  --parallel
  --build_wheel
  --build_shared_lib
  --skip_tests
)

# 6. 共通 CMake 定義
EXTRA_DEFINES=(
  'onnxruntime_BUILD_UNIT_TESTS=OFF'
  'onnxruntime_USE_FLASH_ATTENTION=OFF'
  'onnxruntime_USE_MEMORY_EFFICIENT_ATTENTION=OFF'
  'CMAKE_POLICY_VERSION_MINIMUM=3.5'
)

# 7. CUDA / GPU 構成の動的設定
if [ "$HAS_CUDA" = true ]; then
  echo "[INFO] Architecture: ${ARCH}"
  echo "[INFO] CUDA environment detected. (CUDA: ${CUDA_HOME}, cuDNN: ${CUDNN_HOME})"
  BUILD_ARGS+=(
    --use_cuda
    --cuda_home "${CUDA_HOME}"
    --cudnn_home "${CUDNN_HOME}"
  )

  if [ "$HAS_GPU" = true ]; then
    echo "[INFO] NVIDIA GPU is accessible. Setting CMAKE_CUDA_ARCHITECTURES=native."
    EXTRA_DEFINES+=('CMAKE_CUDA_ARCHITECTURES=native')
  else
    echo "[WARN] NVIDIA GPU is not accessible (e.g. running in docker build)."
    echo "[INFO] Fallback: Setting CMAKE_CUDA_ARCHITECTURES=${TARGET_CUDA_ARCH}."
    EXTRA_DEFINES+=("CMAKE_CUDA_ARCHITECTURES=${TARGET_CUDA_ARCH}")
  fi
else
  echo "[INFO] Architecture: ${ARCH}"
  echo "[INFO] CUDA environment not found. Building CPU-only package."
fi

# 実行
exec ./build.sh "${BUILD_ARGS[@]}" --cmake_extra_defines "${EXTRA_DEFINES[@]}"
