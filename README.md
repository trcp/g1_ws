# G1_ws

## G1 と接続する方法
1. G1 と PC を Ethernet で接続する
1. G1 に SSH する
    ```bash
    ssh unitree@192.168.123.164
    ```
    - パスワードは `123`

## 初回セットアップ方法
1. このリポジトリを G1 のホームディレクトリにインストールしてください．
1. G1 の初期環境で古い Docker を使用しているので，以下のコマンドを実行して Docker を再インストールしてください．
    ```bash
    sudo apt update ; sudo apt install -y ca-certificates curl gnupg lsb-release &&\
    sudo mkdir -p /etc/apt/keyrings &&\
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg &&\
    echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null &&\
    sudo apt update ; sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin &&\
    sudo usermod -aG docker $USER
    ```
    完了したらロボットを再起動してください．
    
1. [Realsense パッケージ](https://github.com/GAI-313/nakalab_realsense/tree/main#) を src ディレクトリにインストールする<br>
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

## Docker を起動する
```bash
docker compose up -d erasers_g1 && docker compose exec erasers_g1 bash
```
