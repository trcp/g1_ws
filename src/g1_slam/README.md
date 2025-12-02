# G1 SLAM

## Usage

### DLiO によるマッピング

> [!IMPORTANT]
> 以下の作業は Docker コンテナ内で実施します．

1. **DLiO を起動する**<br>
    　以下のコマンドを実行して DLiO を起動します．
    ```bash
    ros2 launch g1_slam dlio.launch.py
    ```
    　もし rosbag からマップを作成したい場合は，以下のように `use_sim_time` に `True` を渡してください．
    ```bash
    ros2 launch g1_slam dlio.launch.py use_sim_time:=true
    ```

1. **マップを保存する**<br>
    　以下のコマンドを実行してマップを保存します．このコマンドではカレントディレクトリに `testmap` から始まる PCD マップを保存します．`plefix` を編集することで保存先のパスを変更することができます．
    ```bash
    ros2 run g1_slam pointcloud_to_pcd --ros-args -p input_topic:=/map -p prefix:=$PWD/testmap
    ```

## マップを 3D から 2D マップに変換する

> [!IMPORTANT]
> 以下の作業は Docker コンテナ内で実施します．

1. **保存した pcd マップを確認する**<br>
    　以下のコマンドを実行して保存したマップを確認できます．引数 `pcd_path` に pcd マップファイルまでの絶対パスを記述してください．この時相対パスや `~` などの記号は利用できません．
    ```bash
    ros2 launch g1_slam pcd_map_visualizer.launch.py pcd_path:=$HOME/colcon_ws/map/testmap...
    ```

1. **pcd マップファイルを 2D マップに変換する**<br>
    　以下のコマンドを実行して pcd マップを変換します．第１引数に変換元の pcd マップまでの絶対パスを指定します．
    ```bash
    ~/colcon_ws/build/pointcloud_to_2dmap/pointcloud_to_2dmap ~/colcon_ws/map/testmap... \
        --dest_directory ~/colcon_ws/map/ \
         --min_height -2.0 --max_height 0.1 -r 0.05
    ```
    - `--dest_directory 2d マップファイル保存先のディレクトリまでの絶対パス`
    - `--min-height 変換時に参照する最低高度`
    - `--max-height 変換時に参照する最高高度`
    - `-r 2d マップの解像度`

1. **2D マップの不要部分を削除する**<br>
    　ローカル環境で以下のコマンドを実行するとエディタが開きます．
    ```bash
    docker compose exec devel bash -ic "python3 src/erasers_g1/mapeditor.py"
    ```
