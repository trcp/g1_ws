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
    colcon build --symlink-install --packages-up-to erasers_g1_ros
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
docker compose run --name colcon_build --rm g1 bash -ic "colcon build --symlink-install --packages-up-to erasers_g1_ros --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=humble --cmake-clean-cache"
```
```bash
docker compose run --name colcon_build --rm katana bash -ic "colcon build --symlink-install --packages-up-to erasers_g1_ros --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=humble --cmake-clean-cache"
```

## パッケージ名の対応

G1 用パッケージは `erasers_g1_*` に統一しています．`erasers_g1_api`，`amazing_hand_*`，外部依存パッケージの名前は維持しています．

| 旧名 | 現在の名前 |
| --- | --- |
| `erasers_g1` | `erasers_g1_ros` |
| `erasers_g1_common_cpp` | `erasers_g1_common` |
| `g1_srvs` | `erasers_g1_interfaces` |
| `g1_bringup` | `erasers_g1_bringup` |
| `g1_description` | `erasers_g1_description` |
| `g1_hw_controller` | `erasers_g1_hw_controller` |
| `g1_moveit` | `erasers_g1_moveit` |
| `g1_cartographer` | `erasers_g1_cartographer` |
| `g1_navigation` | `erasers_g1_navigation` |
| `robot_tasks` | `erasers_g1_tasks` |
| `head_servo_controller` | `erasers_g1_head_servo_controller` |
| `person_tracker` | `erasers_g1_person_tracker` |
| `machida_navigation` | `erasers_g1_machida_navigation` |
| `obstacle_detection` | `erasers_g1_obstacle_detection` |

`ros2 run`・`ros2 launch` のパッケージ指定，Python の import，独自インターフェースの型名には現在の名前を使用してください．実行コマンド名，トピック・サービス・Action 名，ロボットの関節名は改名によって変更していません．
