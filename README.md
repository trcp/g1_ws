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
