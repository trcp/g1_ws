# G1_ws

## G1 と接続する方法
1. G1 と PC を Ethernet で接続する
1. G1 に SSH する
    ```bash
    ssh unitree@192.168.123.164
    ```
    - パスワードは `123`

## 初回セットアップ方法
1. Realsense パッケージを src ディレクトリにインストールする<br>
    ```bash
    cd src
    ```
    ```bash
    git clone https://github.com/GAI-313/nakalab_realsense.git
    ```

1. Docker コンテナをビルドする
    ```bash
    # g1_ws
    docker compose build erasers_g1
    ```
