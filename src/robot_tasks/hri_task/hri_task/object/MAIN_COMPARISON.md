# mainとの差分と移植範囲

## 比較基準

調査時の作業ブランチは`hri_task`、HEADは`856f6b1`です。
調査時に取得済みだった`origin/main`は`0d4d74e`です。リモートへのfetchは
実施していないため、最新のmainと比較する場合は参照先を確認してください。

## `erasers_g1_api` importエラーの原因

ホスト側で次のエラーが発生していました。

```text
ModuleNotFoundError: No module named 'erasers_g1_api'
```

ソースがないのではなく、既存の`build/`と`install/`がコンテナ内の
`/home/roboworks/colcon_ws`を絶対パスとして参照していることが原因です。
既存のkatanaコンテナでROSとワークスペースのsetupを読み込むと、
`ArmControl`、`G1Control`、`run_object_grasp.py --help`のimportは成功しました。

今回の実行環境はDockerなので、ホスト側の壊れたsymlinkを直すために
mainのAPIを上書きしたり、ホスト側で再ビルドしたりする必要はありません。

## main方式へ合わせた部分

| 項目 | 現在の実装 |
| --- | --- |
| 腕指令 | `DirectJointController`ではなく`ArmControl.joint_control()` |
| 軌道実行 | `/move_action`からMoveIt・ros2_controlを経由 |
| 上半身制御 | `/enable_upper_body_control`で開始・終了 |
| 頭部 | `G1Control.move_head()` |
| Amazing Hand | `ArmControl.hand_control()` |
| IK | 左右の簡易2リンクIKを比較し、選択腕の関節角をMoveItへ渡す |
| カメラ外部変換 | 暫定寸法ではなく`base_link ← d455_color_optical_frame`のTF |
| 左右関節安全余裕 | URDFの左右別上限から0.10 rad内側を必須化 |

使用している`ArmControl`と`G1Control`の主要メソッドは、調査時のmainと
引数・実装が一致していました。API一式をmainからコピーする必要はありません。

## この機能のソース範囲

物体把持側は次のファイルで構成されます。

```text
src/robot_tasks/hri_task/hri_task/g1_config.py
src/robot_tasks/hri_task/hri_task/object/object_grasp.py
src/robot_tasks/hri_task/hri_task/object/rgbd_geometry.py
src/robot_tasks/hri_task/hri_task/object/run_object_grasp.py
```

RGBDメタデータを生成するため、YOLO側の次の変更も必要です。

```text
src/robot_tasks/hri_task/hri_task/yolo_human/yolo_human_node.py
src/robot_tasks/hri_task/hri_task/yolo_human/docker-compose.yml
```

テストと実機なしの確認ツールは次です。

```text
src/robot_tasks/hri_task/hri_task/object/test_rgbd_geometry.py
src/robot_tasks/hri_task/hri_task/object/test_run_object_grasp.py
src/robot_tasks/hri_task/hri_task/object/check_rgbd_geometry.py
```

`rgbd_geometry.py`は新規ファイルです。未追跡状態では通常の`git diff`に内容が
表示されないため、移植・コミット時に漏らさないでください。

## 差分確認

ワークスペース直下で実行します。

```bash
cd /home/roboworks/g1_ws

git status --short
git diff --check
git diff HEAD -- \
  src/robot_tasks/hri_task/hri_task/object \
  src/robot_tasks/hri_task/hri_task/yolo_human/yolo_human_node.py \
  src/robot_tasks/hri_task/hri_task/yolo_human/docker-compose.yml
```

新規ファイルは`git status --short`と実ファイルを別途確認してください。
現在の作業ツリーへ同じパッチを重ねて適用したり、`git reset --hard`で
作業内容を消したりしないでください。

## 実機で確認できた範囲

- D455のRGB、整列済み深度、CameraInfoを受信
- RGBと深度の撮影stamp一致、およびカメラ時計の固定オフセットを確認
- YOLOの実検出JSONを受信し、appleを7フレームで安定判定
- 到達範囲外の3姿勢を検出し、物体方向へ動かず安全停止
- TOP GRASP HOMEと起動時姿勢の復帰

## 次に実機で確認する範囲

- TFによる実際の`robot xyz`と到達距離（まず`--plan-only`）
- MoveItの軌道計画と実行
- Amazing Handの開閉
- 実機での中央・左・右の到達精度
- カメラ・机・把持物との非接触

実行手順と、ロボットを動かさない`--plan-only`の使い方は
[README.md](README.md)を参照してください。
