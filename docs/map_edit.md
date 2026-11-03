# マップを編集する

> [!CAUTION]
> **前提条件**
> - 事前に手順 [**マップを作成する**](/docs/map_slam.md) を実施している．

## エディタを開き，マップを読み込む方法
1. Katana PC などの外部 PC と G1 を接続し，次のコマンドを実施して **ディスプレイを共有して** G1 へ `ssh` してください．
    ```bash
    ssh -X unitree@192.168.123.164
    ```

1. G1 内部で `g1_ws` に移動します．
    ```bash
    cd g1_ws
    ```

1. 以下のコマンドでマップエディタを開きます．
    ```bash
    python3 src/mapeditor.py
    ```
    　次のようなエディタが起動したら成功です．
    <br><img width=50% src="https://i.imgur.com/qqrrifC.png"/>

1. 「Load」ボタンを押してマップを読み込みます．<br>
    <img width=50% src="https://i.imgur.com/JMWQ77I.png"/>

1. 「Directory」メニューから `map` ディレクトリを指定します．<br>
    <img width=50% src="https://i.imgur.com/qLee6hP.png"/

1. 開きたいマップデータを指定します．この時 yaml，png，pgm 形式のデータをサポートします．<br>
    <img width=50% src="https://i.imgur.com/n88IATp.png"/>

1. 開きたいマップデータを選択したら，「Open」をクリックします．<br>
    <img width=50% src="https://i.imgur.com/vpc4Rd6.png"/>

1. エディタにマップデータが読み込まれます．「OK」ボタンを押してエディタに戻ります．<br>
    <img width=50% src="https://i.imgur.com/WHxNvNJ.png"/>

1. エディタの概要は次のとおりです．<br>
    - `Files`<br>
        ファイルの読み込み，変更内容の保存
    - `Edit`<br>
        編集の取り消し，手順を巻き戻す
    - `Color`<br>
        使用するマップのカラーを選択
    - `Tool`<br>
        使用するペンシルツールの種類を選択
    - `View / Size`<br>
        ペンシルのサイズとプレビューサイズの変更
    <img width=50% src="https://i.imgur.com/5UAzArx.png"/>


## 編集方法
　例として，障害物の線を引く方法を説明します．

1. 「Color」プロパティから「Wall」（障害物領域）を選択します．<br>
1. 「Tools」プロパティから「Line」（直線ツール）を選択します．<br>
1. 開始地点をクリックします．この時画面に青い直線プレビューが表示されます．<br>
1. 終了地点をクリックします．<br>
    <img width=50% src="https://i.imgur.com/qTByshn.png"/>

1. 障害物領域を直線で引くことができます．<br>
    <img width=50% src="https://i.imgur.com/vyz6dgi.png/>

## 保存方法
1. 「Save」ボタンをクリックします．<br>
    <img width=50% src="https://i.imgur.com/Mnvid6n.png"/>

1. ダイアログが表示されたら「Save」をクリックします．この時ファイル名は変更しないでください．<br>
    <img width=50% src="https://i.imgur.com/JSsAyw8.png"/>

1. 上書き保存の確認画面が表示されたら「Yes」をクリックします．<br>
    <img width=50% src="https://i.imgur.com/vfldZyz.png"/>

1. 保存成功画面が出たら成功です．<br>
    <img width=50% src="https://i.imgur.com/mAJGnc7.png"/>

1. ウィンドウを閉じて作業を終了します．<br>
    <img width=50% src="https://i.imgur.com/idLjTmr.png"/>

---

- [🏡 README にもどる](/README.md)
