# onnxruntime をビルドする
1. ワークスペースに移動します．
    ```bash
    cd onnxruntime
    ```
1. 以下のコマンドを実行してビルド環境を作成します．
    ```bash
    docker buildx build --platform=linux/arm64 -t onnxruntime:onboard .
    ```
    ```bash
     docker buildx build --platform=linux/amd64 -t onnxruntime:secondary --build-arg TARGET=secondary .
    ```
1. 次のコマンドでビルドを行います．NvidiaContainerToolkit を利用する場合は `--runtime nvidia` を追加します．
    - `Nvidia GPU` を使用する場合
        ```bash
        docker run --rm --runtime nvidia -v ${PWD}/build:/tmp/onnxruntime/build onnxruntime:onboard
        ```
        ```bash
        docker run --rm --runtime nvidia -v ${PWD}/build:/tmp/onnxruntime/build onnxruntime:secondary
        ```
    - `Nvidia GPU` を使用しない場合
        ```bash
        docker run --rm -v ${PWD}/build:/tmp/onnxruntime/build onnxruntime:onboard
        ```
        ```bash
        docker run --rm -v ${PWD}/build:/tmp/onnxruntime/build onnxruntime:secondary
        ```

1. C++ 開発用ヘッダファイル（`include`）をイメージから抽出します．
    ```bash
    docker run --rm onnxruntime:onboard tar -C /tmp/onnxruntime -cf - include | tar -xf -
    ```
    ```bash
    docker run --rm onnxruntime:secondary tar -C /tmp/onnxruntime -cf - include | tar -xf -
    ```
