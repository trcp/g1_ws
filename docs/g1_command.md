# G1 操作コマンド一覧

　常用する Unitree G1 の操作コマンドを解説します．

- SSH（パスワードは `123` です．）
    ```bash
    ssh unitree@192.168.123.164
    ```
- ワンライナー起動
    ```bash
    ssh -t unitree@192.168.123.164 \
    "cd g1_ws && docker compose run --rm --name erasers_g1_bringup erasers_g1 bash -ic \
    \"ros2 launch erasers_g1_bringup bringup.launch.py\""
    ```

- JTOP
    ```bash
    ssh -t unitree@192.168.123.164 jtop
    ```
---

- [🏡 README にもどる](/README.md)
