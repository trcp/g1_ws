# g1_ws

<details>
<summary>クライアント PC セットアップ</summary>

　クライアント PC（G1 と接続する PC）の g1_ws をビルド方法を解説します。

1. このリポジトリをクローンする
1. 環境変数 `PASSWORD` にコンテナ内部アカウント用のパスワードを設定する。ローカリューザーと同じパスワードにすると良い。
    ```bash
    export PASSWORD=password
    ```
1. `katana` サービスをビルドする。
    ```bash
    docker compose build katana
    ```
1. Docker イメージ `erasers_g1:latest` がビルドされたか確認する。
    ```bash
    docker images | grep erasers_g1
    ```

</details>


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

<details>
<summary>ジョイント操作について</summary>

### ジョイント操作について

`erasers_g1` コンテナ起動時に別のターミナルで以下のコマンドを実行して joint_state_publisher_gui を起動します．

- `erasers_g1` コンテナ内の場合
    ```bash
    ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args -r __node:=joint_state_publisher_gui -r /joint_states:=/upper_joints_control
    ```
- ホストからの場合
    ```bash
    docker compose exec erasers_g1 bash -ic "ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args -r __node:=joint_state_publisher_gui -r /joint_states:=/upper_joints_control"
    ```

ノードが起動するとアームが初期姿勢をとり，joint_state_gui から上半身のジョイントを操作できるようになります．

<img width=100% src="https://i.imgur.com/g5LCEWM.jpeg" />

> [!CAUTION]
> **決して joint_state_gui の `Randamize` を押さないでください！アームが予測しない姿勢をとる恐れがあります．**

> [!NOTE]
> `Center` ボタンをクリックするとアームと頭部カメラは初期姿勢になります．

- ノードを修了するなどして，`JointState` メッセージの Publish が止まるとロボットは歩行姿勢に戻ります．

</details>

<details>
<summary>エンドエフェクタ制御について</summary>

### エンドエフェクタ制御について

以下のコマンドを実行してエンドエフェクタの制御を有効にします．
```bash
ros2 service call /enable_ee_control std_srvs/srv/SetBool "data: true"
```
RViz2 からインタラクティブマーカーを使いエンドエフェクタを移動できます．

---

以下のコマンドを実行してエンドエフェクタの制御を無効にします．
```bash
ros2 service call /enable_ee_control std_srvs/srv/SetBool "data: false"
```

---

以下のコマンドでエンドエフェクタの姿勢を初期姿勢に戻すことができます．
```bash
ros2 service call /set_init_pose std_srvs/srv/Trigger
```

---

以下のコマンドで両方のエンドエフェクタの姿勢を考慮します．
```bash
ros2 service call /enable_dual_arm_ik std_srvs/srv/SetBool "data: true"
```

---

以下のコマンドで2tのエンドエフェクタの姿勢を同期して IK を解くようになります．両手でものを持つときに便利です．
```bash
ros2 service call /sync_ee_pose std_srvs/srv/SetBool "data: true"
```

</details>

<details>
<summary>SLAM について</summary>

### SLAMについて

G1 内部で以下のコマンドを実行して 2D マップを作成．[map](/map) ディレクトリに自動保存されます．
```bash
docker compose exec erasers_g1 bash -ic "ros2 launch g1_cartographer slam_toolbox.launch.py"
```

</details>
