FROM ros:humble-ros-base

# OpenCVやROSの依存パッケージをインストール
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    git \
    && rm -rf /var/lib/apt/lists/*

# YOLOv8、OpenCV、そしてCLIP（競合回避のため複数指定）をインストール
RUN pip3 install --no-cache-dir ultralytics opencv-python "numpy<2" openai-clip "lap>=0.5.12"

WORKDIR /app

# Pythonのstdoutバッファリングを無効化（docker logsにリアルタイムで出力）
ENV PYTHONUNBUFFERED=1
# ホストと同じDDS（CycloneDDS）を使うよう明示的に設定
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROSの環境を読み込んでからPythonスクリプトを直接実行
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /app/yolo_human_node.py"]
