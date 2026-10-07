# Object Grasp

YOLOで指定物体を検出し、簡易2リンクIKで左右両腕の関節値を計算した後、
選択した腕の関節値をMoveItで実行します。

このディレクトリの役割は次の3つです。

1. RGB・整列済み深度・CameraInfoから、安定した物体座標を得る
2. 右腕と左腕の手前待機・横把持・リフト関節値を計算する
3. 右腕を優先し、右が計算上届かない場合だけ左腕を選んでMoveItへ送る

## ファイル構成

- `object_grasp.py`: YOLO結果の検証、安定判定、座標変換、簡易IK
- `rgbd_geometry.py`: RGBDの時刻・座標系・校正値の共通検証
- `run_object_grasp.py`: MoveIt、頭部、音声、ハンドを含む実行フロー
- `check_rgbd_geometry.py`: 実機なしで中央・左・右の投影を確認するツール
- `test_rgbd_geometry.py`: RGBDと安定判定の単体テスト
- `test_run_object_grasp.py`: 動作順序・音声・速度設定の模擬テスト
- `MAIN_COMPARISON.md`: `main`との差分と移植対象

`DirectJointController`と、アプリからの`/upper_joints_control`直接publishは使用しません。

## 処理の流れ

```text
--target省略時は英語で対象名を質問
→ MoveIt上半身制御を有効化
→ 起動時の関節姿勢と頭部チルトを保存
→ 両手を閉じる（カメラ下降・認識中は腕を動かさない）
→ YOLOで安定した物体座標を取得
→ TFからbase_link←カメラ光学座標系の実機変換を取得（無ければ中止）
→ 左右それぞれの物体手前10cm・把持点・把持後上方10cmを計算
→ 右腕の3姿勢が到達可能なら右腕を選ぶ
→ 右腕が到達不可で左腕が到達可能な場合だけ左腕を選ぶ
→ 両腕とも距離近似だけが範囲外なら、旧git版と同じ境界丸め関節値の右腕を選ぶ
→ 選択腕だけをTOP GRASP HOMEへ移動
→ 選択した手を開く
→ 親指を下へ向けた選択手を物体手前10cmへ移動
→ Y/Zを変えずX方向へ横移動して手を閉じる
→ X/Yを変えず10cm上へリフト
→ 選択腕をTOP GRASP HOMEへ戻す
→ 閉状態を確認してから腕を起動時姿勢へ下げる
→ 頭部を復元
→ MoveIt上半身制御を無効化
```

復帰はHOME関節角への再計画です。行きの軌道を記録して逆再生する方式ではありません。
各区間は関節角目標へのMoveIt計画であり、手先が完全な直線を通る
Cartesian Pathではありません。把持後のリフトに失敗した場合は、机上を横引きしないよう
自動HOME復帰・頭部復元・上半身制御解除を止め、オペレーターの復旧を待ちます。

## 必要なROSインターフェース

通常の把持動作には次が必要です。

```text
/joint_states
/move_action
/enable_upper_body_control
/upper_body_controller/follow_joint_trajectory
/move_servo
/tf
/tf_static
/yolo_human/command
/yolo_human/result
```

Amazing Handを`real`で使う場合は`/hand_command`も必要です。

bringupや`arm_joint_control`を重複起動しないでください。このスクリプトと
`DirectJointController`を同時に動かすと、低層の腕指令が競合します。

## 実行環境

既存の`build/`と`install/`は、katanaコンテナ内の
`/home/roboworks/colcon_ws`で生成されています。ホスト側の仮想環境ではなく、
既存のkatanaコンテナ内でROSとワークスペースのsetupを読み込んで実行します。

```bash
# ホスト側
cd /home/roboworks/g1_ws
docker compose exec katana bash

# katanaコンテナ内
source /opt/ros/humble/setup.bash
source /home/roboworks/colcon_ws/install/setup.bash
cd /home/roboworks/colcon_ws/src/robot_tasks/hri_task/hri_task/object
```

`dummy`や`disabled`でも、`ArmControl`のimportに
`amazing_hand_interfaces`のビルド済みサービス型が必要です。ハンド本体と
`amazing_hand_node`は`real`の場合だけ必要です。

## 実機を動かさない確認

### 1. 単体テスト

ROS通信、カメラ、ロボットを使わずに実行できます。

```bash
python3 -B -m unittest -q test_rgbd_geometry.py test_run_object_grasp.py
```

### 2. 中央・左・右の投影確認

既定では従来の固定内部パラメータを使用します。

```bash
python3 check_rgbd_geometry.py
```

CameraInfoの値が分かる場合は、実測値でも確認できます。

```bash
python3 check_rgbd_geometry.py \
  --fx 615.2 --fy 615.0 --cx 319.8 --cy 239.9 --depth 0.5
```

これは数式と左右方向を確認するテストです。実機カメラの校正精度までは保証しません。

### 3. YOLO compose設定の確認

```bash
cd /home/roboworks/g1_ws/src/robot_tasks/hri_task/hri_task/yolo_human
docker compose config --quiet
```

### 4. YOLOコンテナへの反映

`rgbd_geometry.py`を追加マウントしているため、単純なrestartではなく再作成します。
共有YOLOを使う別タスクが停止しているときに実行してください。

```bash
docker compose up -d --no-deps --force-recreate yolo_human
docker compose logs --tail=80 yolo_human
```

既定のCycloneDDS設定は実機通信用の`enp3s0`を使います。実機なしのPCでこのIFが
停止している場合だけ、DDSの自動選択で起動します。

```bash
YOLO_CYCLONEDDS_URI= \
  docker compose up -d --no-deps --force-recreate yolo_human
```

確認するログは次のとおりです。

- RGB画像を受信した
- 整列済み深度画像を受信した
- CameraInfoを利用できている、または固定値へのフォールバックが表示された
- `rgbd_geometry`のimportエラーがない

### 5. 腕を動かさない認識・座標計算

`--plan-only`では`ArmControl`、`G1Control`、ハンド制御を生成せず、
YOLO検出、安定判定、座標変換、左右それぞれの手前待機・横把持・リフト計算と
右腕優先の候補選択を表示します。座標変換には通常実行と同じ
`base_link ← d455_color_optical_frame`のTFを使います。

```bash
python3 run_object_grasp.py \
  --target orange \
  --plan-only
```

中央・左・右へ同じ距離の物体を置き、`camera xyz`、`robot xyz`、
`arm`、`shoulder @0`、`arm distance`、`waist_yaw_joint`を比較してください。
`shoulder @0`は腰が正面のときの肩から目標までの3次元直線距離です。
`arm distance`と`reachable`は平面2リンク近似による選択値です。
通常実行でも`--plan-only`と同じ関節値を使い、MoveItにはIK計算を依頼しません。
`coordinates`は必ず
`tf:base_link<-d455_color_optical_frame`になっていることを確認します。
TFが取得できない場合はIKを作らず終了します。

## 通常実行

bringup、MoveIt、YOLOを起動した後に実行します。

```bash
# 既定では実機Amazing Handを開閉
python3 run_object_grasp.py --target orange

# ハンド指令をログ表示だけにする
python3 run_object_grasp.py --target orange --hand-mode dummy

# ハンド処理を完全に省略
python3 run_object_grasp.py --target orange --hand-mode disabled
```

`--target`を省略すると、HRI Taskと同じ`TTS`と`SpeechToText`を使用します。
ピン音の後に`apple`、`bottle`、`orange`、`banana`のいずれかを英語で答えます。
無音・候補外・複数候補は最大3回まで聞き直し、対象が決まらない場合は動作しません。

```text
Please say the name of the object you would like me to pick up after the beep.
```

音声入力には`/play_audio`、`/mic_rec`、`/audio/raw`、Whisperモデル、
音声認識のPython依存が必要です。`--target`指定時は音声依存を読み込みません。

## RGBDの検証と安定判定

bringupファイルは変更しません。YOLOは次を購読します。

```text
/head_camera/d455/color/image_raw
/head_camera/d455/aligned_depth_to_color/image_raw
/head_camera/d455/color/camera_info
```

整列済み深度が届かない場合だけ、YOLOノードから
`/head_camera/d455/set_parameters`へ`align_depth.enable=true`を最大3回要求します。
未整列の深度画像へフォールバックすることはありません。

RGBと深度には次の条件を適用します。

- 撮影時刻差50ms以内
- YOLOノードで受信してから2秒以内
- 画像サイズが一致
- 光学座標系が一致
- 同じ画像を再利用しない
- `16UC1`はmmからmへ変換、`32FC1`はmとして使用

RGB・深度間の同期と重複判定にはD455の撮影stampを使用します。鮮度と把持要求後の
画像かどうかはYOLOノードの受信時刻で判定するため、D455とROSシステム時刻に
固定オフセットがあっても、停止画像や過去結果を再利用しません。

CameraInfoを取得できた場合は、焦点距離、主点、歪み係数を使用します。
未取得または未校正の場合に限り、640×480用の固定値
`FX=FY=615`、`CX=320`、`CY=240`を使用します。

カメラ座標からロボット座標への変換は、コード内の暫定的な首・カメラ寸法ではなく、
実機のTFを使用します。これにより、D455の取付位置、光学座標軸、現在の頭部チルトを
1つの剛体変換として扱います。画像stampには実機で固定の時刻差が確認されているため、
頭部を静止させてから取得した最新TFを使用します。
G1 URDFの`d455_joint`は正方向が下向きで、固定取付角`0.1 rad`もTFに含まれます。
そのため旧コードの固定7度補正は追加せず、チルトを二重に加算しません。

物体座標は次の条件をすべて満たしてから確定します。

- 異なる5フレーム以上
- 撮影期間0.5秒以上
- 各フレーム間隔0.5秒以内
- bbox中心の範囲が各軸6画素以内
- bboxが画像内の有効な矩形である（画像端への接触・見切れは許可）
- カメラ座標X/Y/Zの範囲が各軸2cm以内
- track ID、またはbboxの重なりで同じ対象を追跡できている

安定区間のbboxと深度の中央値を把持計算へ渡します。対象が動く、見失われる、
別対象へ切り替わる、校正値が変わる場合は安定区間を取り直します。
対象が画像端で切れていても、見えているbbox中心と整列済み深度から座標を計算します。

## 調整値

[run_object_grasp.py](run_object_grasp.py)の主な設定は次のとおりです。

```python
DETECTION_TIMEOUT_SEC = 8.0
ARM_VELOCITY_SCALE = 0.5
ARM_ACCELERATION_SCALE = 0.5
MOTION_PAUSE_SEC = 1.0
SIDE_APPROACH_CLEARANCE_M = 0.10
POST_GRASP_LIFT_M = 0.10
GRASP_OFFSET_Z_M = 0.0
ROBOT_BASE_FRAME = "base_link"
CAMERA_TF_TIMEOUT_SEC = 3.0
TOP_GRASP_ELBOW_RAD = -0.90
```

安定判定の閾値は[object_grasp.py](object_grasp.py)の`STABLE_*`、
画像時刻の条件は[rgbd_geometry.py](rgbd_geometry.py)にあります。
左右の関節上限はG1 URDF値を個別に使用します。TOP GRASP HOMEには上下限から
`0.10 rad`（約5.7度）の余裕を要求し、把持用の簡易関節値はURDFのハードリミット内で
あることを確認します。肩ピッチ・肘は左右同符号、肩ロールと横把持用手首ロールは
左右で反転します。実機で親指が上になることを確認したWALK姿勢の手首角、
右`-0.13 rad`、左`+0.08 rad`を使用します。

## 現在の制約

- 把持IKは腕を平面2リンクとして近似し、MoveItは送られた関節値の軌道計画と実行を
  担当します。
- 実機実行と`--plan-only`はカメラTFを必須とし、取得できなければ物体方向へ動きません。
- 右腕を基本とし、右腕の3姿勢が簡易計算で到達不可の場合に左腕を使います。
- 横接近とリフトは関節空間の複数目標であり、完全な直線軌道ではありません。
- Amazing Handの衝突形状はMoveItモデルへ登録されていません。
- 手・肘と頭部カメラの衝突判定には無効化された組み合わせがあります。
- 机・把持物をPlanning Sceneへ登録していません。
- 把持成否の判定はありません。

したがって、MoveItの計画成功だけでは、カメラや机との非接触を保証できません。
