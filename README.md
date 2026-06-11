# g1_ws

1. Whisper モデルをダウンロード
    ```bash
    cd src/erasers_g1_api/config &&\
    git clone https://huggingface.co/Systran/faster-whisper-small &&\
    cd -
    ```
1. 依存関係パッケージのダウンロード
    ```bash
    vcs import ./src/thirdparty/ < depends.repos
    ```
1. 依存関係を自動解決
    ```bash
     sudo apt update && rosdep install -y -i --from-path .\
        --skip-keys "pointcloud_to_2dmap pcl_localization_ros2 direct_lidar_inertial_odometry fast_lio lightweight_openpose_ros2 sam3_ros"
    ```
1. [GLIM](https://koide3.github.io/glim/installation.html) をインストール

1. ワークスペースをビルドする
    ```bash
    colcon build --symlink-install --packages-up-to erasers_g1
    ```

1. chrony をインストールする
    ```bash
    sudo apt install -y chrony
    ```

1. chrony 設定ファイルをコピーする
    ```bash
    sudo cp chrony.conf /etc/chrony/chrony.conf
    ```

1. chrony を再起動する
    ```bash
    sudo service chrony restart
    ```

1. `192.168.123.161` と時刻同期できているか確認する
    ```bash
    chronyc sources
    ```


---

```bash
docker compose run --name colcon_build --rm g1 bash -ic "colcon build --symlink-install --packages-up-to erasers_g1 --cmake-args -DROS_ED
ITION="ROS2" -DHUMBLE_ROS=humble --cmake-clean-cache"
```
```bash
docker compose run --name colcon_build --rm katana bash -ic "colcon build --symlink-install --packages-up-to erasers_g1 --cmake-args -DROS_ED
ITION="ROS2" -DHUMBLE_ROS=humble --cmake-clean-cache"
```