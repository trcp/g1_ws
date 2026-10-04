# katana（開発用 laptop）

> [!CAUTION]
> 前提条件
> - **NVIDIA GPU** が搭載されている
> - **NVIDIA GPU ドライバ** がインストールされている

### Onnxruntime をビルド

> [!CAUTION]
> - この作業の完了には **長時間** かかります．
> - `onnxruntime` のビルドには **大量のメモリ，CPU プロセス** を消費します．他のプロセスは事前に止めておくことをおすすめします．

1. `onnxruntime` ディレクトリに移動する
    ```bash
    cd ./onnxruntime
    ```
1. `onnxruntime` ビルド環境を作成する
    ```bash
    docker buildx build --platform=linux/amd64 -t onnxruntime:secondary --build-arg TARGET=secondary .
    ```
1. `onnxruntime` をビルドする
    ```bash
    docker run --rm --runtime nvidia -v ${PWD}/build:/tmp/onnxruntime/build onnxruntime:secondary
    ```
1. ビルドされた `onnxruntime` ファイルをローカルに持ってくる
    ```bash
    docker run --rm onnxruntime:secondary tar -C /tmp/onnxruntime -cf - include | tar -xf -
    ```

---

- [🏡 README にもどる](/README.md)
