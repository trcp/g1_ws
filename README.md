# eR@sers G1 Workspace

## G1（ロボット実機）
　前提として，以下のコマンドを実施して実機に SSH してログインしてください．
```bash
ssh unitree@192.168.123.164
```

> [!NOTE]
> パスワードは `123` です．

### Onnxruntime をビルド

> [!CAUTION]
> - この作業の完了には **長時間** かかります．
> - `onnxruntime` のビルドには **大量のメモリ，CPU プロセス** を消費します．他のプロセスは事前に止めておくことをおすすめします．

1. `onnxruntime` ディレクトリに移動する
    ```bash
    cd ./onnxruntime
    ```
1. `onnxruntime` ビルド環境を作成する
    ```bash
    docker buildx build --platform=linux/arm64 -t onnxruntime:onboard .
    ```
1. `onnxruntime` をビルドする
    ```bash
    docker run --rm --runtime nvidia -v ${PWD}/build:/tmp/onnxruntime/build onnxruntime:onboard
    ```
1. ビルドされた `onnxruntime` ファイルをローカルに持ってくる
    ```bash
    docker run --rm onnxruntime:onboard tar -C /tmp/onnxruntime -cf - include | tar -xf -
    ```

### `erasers_g1` コンテナをビルド

> [!CAUTION]
> - この作業の完了には **長時間** かかります．
> - `erasers_g1` のビルドには **大量のメモリ，CPU プロセス** を消費します．他のプロセスは事前に止めておくことをおすすめします．

1. `g1_ws` 直下で `erasers_g1` コンテナをビルドします．
    ```bash
    docker compose build erasers_G1
    ```
1. コンテナ中の ROS2 パッケージをビルドします．
    ```bash
    docker compose run --name colcon_build --rm erasers_g1 bash -ic \
        "colcon build --symlink-install --packages-up-to erasers_g1_ros --cmake-clean-cache \
        --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=humble"
    ```
