# YOLO Human (HRI Task)

このディレクトリはYOLOを用いた人物検出およびOpenAI APIを利用した解析を行うためのモジュールです。
Dockerコンテナ上で動作させることを前提としています。

## 環境変数の設定手順（APIキーの引き継ぎ）

本モジュールはOpenAIのAPIを利用するため、APIキーが必要です。
セキュリティ上、APIキーはファイルに直書きせず、PC（ホスト側）のbash環境変数（`~/.bashrc` など）から読み込む構成になっています。

以下の手順で、ホストPCのAPIキーをDocker内に引き継ぐための `.env` ファイルを作成してください。

### 1. `.env.example` をコピーして `.env` を作成する
ターミナルで以下のコマンドを実行し、テンプレートファイルをコピーします。

```bash
cd /home/roboworks/g1_ws/src/robot_tasks/hri_task/hri_task/yolo_human/
cp .env.example .env
```

### 2. PC本体（ホスト側）に環境変数が設定されているか確認する
ホストPCの `~/.bashrc` 等に以下のようにAPIキーが設定されている必要があります。
設定されていない場合は追記してください。

```bash
# ~/.bashrc に以下を追記
export OPENAI_API_KEY="sk-xxxx...（実際のキー）"
```

作成した `.env` ファイルの中身は以下のようになっており、ホストの `OPENAI_API_KEY` を自動的にDocker内に引き継ぐ設定になっています。

**(.env の中身)**
```env
OPENAI_API_KEY=${OPENAI_API_KEY}
```

これで、キーをファイルに直書きすることなく安全にDockerコンテナを起動できます。

## 重みファイルの取得について

YOLOE などの重みファイルはコンテナ内で自動取得され、ホストの `~/.cache` を
`/root/.cache` にマウントしてキャッシュします。`docker-compose.yml` では
`yoloe-26x-seg.pt` や `mobileclip2_b.ts` を個別に bind mount しません。

ホスト側に存在しないファイルを `./file:/app/file` として mount すると、Docker は
ホスト側の `./file` をディレクトリとして作成してしまい、重みのダウンロード先が
壊れます。新しいデバイスでその状態になった場合は、コンテナを停止してから以下を
実行してください。

```bash
docker compose down
sudo rm -rf yoloe-26x-seg.pt mobileclip2_b.ts
docker compose up --build
```
