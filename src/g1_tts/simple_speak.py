import subprocess

# ==========================================
# 喋らせたい言葉
TEXT_TO_SPEAK = "This is a simple test using espeak."
# ==========================================

def run_espeak():
    print(f"読み上げ中: {TEXT_TO_SPEAK}")
    
    # espeakコマンドを直接実行
    # -ven+f3 : 英語の女性風の声
    # -s150   : 読み上げ速度
    cmd = ['espeak', '-ven+f3', '-s150', TEXT_TO_SPEAK]
    
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print("エラー: espeakがインストールされていません。")
        print("sudo apt install espeak でインストールしてください。")

if __name__ == "__main__":
    run_espeak()
