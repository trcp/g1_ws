# G1_ws

## Docker (Host)
### Setup
　カレントディレクトリで以下のコマンドを実行する
```bash
docker compose build g1
```

### Bringup
　以下のコマンドを実行してコンテナを起動する.
```bash
xhost +
docker compose up -d g1
```

## Docker (G1)
### Setup
```bash
ssh unitree@192.168.123.164
```
　カレントディレクトリで以下のコマンドを実行する
```bash
docker compose build g1
```

### Bringup
　以下のコマンドを実行してコンテナを起動する.
```bash
xhost +
docker compose up -d g1 && docker compose exec g1 bash
```

### Bringup MID-360
```bash
docker compose exec g1 bash -i -c "ros2 launch livox_bringup bringup_launch.py use_rviz:=false config_json_path:=/assets/mid360.robot.json"
```
