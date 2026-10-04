# katana（開発用 laptop）

> [!CAUTION]
> 前提条件
> - **NVIDIA GPU** が搭載されている
> - **NVIDIA GPU ドライバ** がインストールされている

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
    docker buildx build --platform=linux/amd64 -t onnxruntime:secondary --build-arg TARGET=secondary .
    ```
1. `onnxruntime` をビルドする
    ```bash
    docker run --rm --runtime nvidia -v ${PWD}/build:/tmp/onnxruntime/build onnxruntime:secondary
    ```
1. ビルドされた `onnxruntime` ファイルをローカルに持ってくる
    ```bash
    docker run --rm onnxruntime:secondary tar -C /tmp/onnxruntime -cf - include | tar -xf -
    ```

## `katana` Docker 環境の設定をする

1. 以下のコマンドを実施して，搭載されている NVIDIA GPU の **アーキテクチャ番号** を取得する
    ```bash
    nvidia-smi --query-gpu=name,compute_cap --format=csv
    ```
    > 以下のようなログが出力されたときの末尾の番号 $`\times 10`$ の値が **NVIDIA GPU アーキテクチャ番号** です．
    > ```
    > NVIDIA GeForce RTX 4090, 8.9
    > ```
    > アーキテクチャ番号：$`8.9 \times 10 = 89`$

1. [`compose.yaml`](/compose.yaml) を編集する<br>
    `compose.yaml` の **69 行付近** の `# Arch settings` 項目の値を編集し，開発環境を設定します．
    |変数名|概要|
    |:---:|:---|
    |`WORKSPACE_TYPE`|以下の２つの任意の文字列を選択してください．<br>- `base` : GUI ツール（`rviz2`）などがインストールされない軽量の開発環境．<br>- `desktop` : GUI ツールがインストールされる GUI 対応環境．|
    |`CTRANSLATE_ARCH`|前回の作業でもとめた **NVIDIA アーキテクチャ番号** を代入してください．|

## `katana` コンテナをビルド

> [!CAUTION]
> - この作業の完了には **長時間** かかります．
> - `katana` のビルドには **大量のメモリ，CPU プロセス** を消費します．他のプロセスは事前に止めておくことをおすすめします．

1. `katana` コンテナ内のユーザーパスワードを設定する．ホストユーザーのパスワードと同じにするといい
    ```bash
    export PASSWORD=<your password>
    ```
1. `g1_ws` 直下で `katana` コンテナをビルドします．
    ```bash
    docker compose build katana
    ```
1. コンテナ中の ROS2 パッケージをビルドします．
    ```bash
    docker compose run --name colcon_build --rm katana bash -ic '
    colcon build \
      --symlink-install \
      --packages-up-to erasers_g1_ros \
      --parallel-workers 1 \
      --executor sequential \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
        -DCMAKE_CXX_FLAGS="-g0 -O1 --param ggc-min-expand=20 --param ggc-min-heapsize=32768" \
        -DCMAKE_C_FLAGS="-g0 -O1 --param ggc-min-expand=20 --param ggc-min-heapsize=32768" \
        -DCMAKE_EXE_LINKER_FLAGS="-Wl,--no-keep-memory -Wl,--reduce-memory-overheads" \
        -DCMAKE_SHARED_LINKER_FLAGS="-Wl,--no-keep-memory -Wl,--reduce-memory-overheads"
    '
    ```

---

- [🏡 README にもどる](/README.md)
