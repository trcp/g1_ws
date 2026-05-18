from huggingface_hub import snapshot_download

# 使用したいモデルのリポジトリ名（smallを指定）
repo_id = "Systran/faster-whisper-small"

# 保存先のローカルディレクトリを指定
local_dir = "../config/faster-whisper-small"

print(f"モデル '{repo_id}' のダウンロードを開始します...")
snapshot_download(repo_id=repo_id, local_dir=local_dir)
print(f"ダウンロード完了: {local_dir}")