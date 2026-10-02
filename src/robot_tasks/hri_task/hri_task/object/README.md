# Object Grasp

音声認識などから得た物体名をYOLOへ渡し、検出位置から右腕の簡易IKを計算して、確認用に腕を動かすコードを配置するディレクトリです。

## 想定する構成

```text
hri_task/hri_task/
├── direct_joint_control.py
├── g1_config.py
├── object/
│   ├── README.md
│   ├── object_grasp.py
│   └── run_object_grasp.py
└── yolo_human/
```

現在`test`ディレクトリにある次の2ファイルを、このディレクトリへ移動して使用します。

```text
object_grasp.py
run_object_grasp.py
```

`object_grasp.py`は親ディレクトリの`g1_config.py`を参照します。`run_object_grasp.py`は同じディレクトリの`object_grasp.py`と、親ディレクトリの`direct_joint_control.py`を使用します。

## 前提

実行前に次のシステムを起動してください。

1. G1 bringup
2. `hri_task/yolo_human`のYOLOコンテナ

使用する主なROSトピックは以下です。

```text
/joint_states
/upper_joints_control
/yolo_human/command
/yolo_human/result
/head_camera/d455/color/image_raw
/head_camera/d455/depth/image_rect_raw
```

## 実行方法
docker compose up katana
source install/setup.bash
した後に
```bash
cd /home/roboworks/g1_ws/src/robot_tasks/hri_task/hri_task/object
python3 run_object_grasp.py --target banana --timeout 15
```

`--target`を省略した場合は、実行時に物体名を入力します。

## 把持点の指定

一般物体では、YOLOのバウンディングボックス中心を狙います。

```bash
python3 run_object_grasp.py --target banana --grasp-strategy center
```

従来のバッグ把持と同じく、物体上部の少し右側を狙う場合は`bag`を指定します。

```bash
python3 run_object_grasp.py --target bag --grasp-strategy bag
```

## 頭部角度

このコードは頭部を動かしません。`--head-tilt`には、認識時の実際の頭部チルト角をラジアンで指定します。頭部が水平ならデフォルトの`0.0`を使用します。

```bash
python3 run_object_grasp.py --target bag --grasp-strategy bag --head-tilt -0.5
```

## 腕の動作順序

```text
起動時の初期姿勢を保存
→ バッグ把持用HOME（右腕を畳む）
→ 計算した把持姿勢
→ バッグ把持用HOME
→ 保存した初期姿勢
```

ハンドの開閉はまだ行いません。

## 主なオプション

```text
--target NAME                 YOLOへ渡す英語の物体名
--timeout SEC                認識タイムアウト。デフォルト8秒
--head-tilt RAD              現在の実際の頭部角度。デフォルト0.0
--grasp-strategy center|bag  把持点の選択。デフォルトcenter
```

## エラー時の確認

### YOLO node is not connected

YOLOコンテナの起動状態と、次のトピックを確認します。

```bash
ros2 topic info /yolo_human/command
ros2 topic info /yolo_human/result
```

### no result message was received

YOLOコンテナのログで、`First RGB image received!`と`First Depth image received!`が出ているか確認します。

### Arm is not ready

G1 bringupの起動状態と、次のトピックを確認します。

```bash
ros2 topic info /joint_states
ros2 topic info /upper_joints_control
```

`/joint_states`から起動時姿勢を取得できない場合、安全のため腕への動作指令は送りません。

## 注意

- 実機の緊急停止を使用できる状態で実行してください。
- 現在のIKは既存のバッグ把持と同じ簡易2リンクIKです。
- 到達範囲外では、従来ロジックと同じく到達限界へ丸めた姿勢を送ります。
- 物体の形状や向きに応じた把持姿勢、衝突回避、把持確認はまだ実装していません。
