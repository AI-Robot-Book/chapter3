# speech_service
## 概要
３章のサンプルプログラム  
音声認識と音声合成を実行するプログラム

## インストール
- オーディオ関連を扱うためのライブラリを以下のコマンドでインストールします．
```
apt install portaudio19-dev
apt install pulseaudio
```
- Pythonのモジュールとして呼びたすために，以下のコマンドを実行します．
```
pip3 install pyaudio
```
- 音声認識ライブラリを以下のコマンドでインストールします．
```
pip3 install SpeechRecognition
```
- サンプルプログラムを以下のコマンドでGitHubからクローンします．
```
cd ˜/airobot_ws/src
git clone https://github.com/AI-Robot-Book/chapter3
```
- 以下のコマンドでパッケージをビルドします．
```
cd ˜/airobot_ws
colcon build
```

## 実行
- 音声認識の実行手順（3.1節）
  - 端末を開いて /speech トピックへパブリッシュしたデータを見られるようにします．
    ```
    ros2 topic echo /speech
    ```
  - 新しい端末を開いて音声認識を起動します．
    ```
    ros2 run speech_service recognition
    ```
  - マイクに向かって発話します．
  
- 音声合成の実行手順（3.2節）
  - 端末を開いて音声合成を起動します．
    ```
    ros2 run speech_service synthesis
    ```
  - 新しい端末を開いて発話させたいメッセージを /speech に送ります．
    ```
    ros2 topic pub -1 /speech std_msgs/msg/String "{data: 'I will go to the kitchen and grab a bottle.'}
    ```
  - スピーカから音声が出力されます．

- トピック通信によるオウム返しの実行手順（3.3.1節）
  - 端末を開いて音声合成を起動します．
    ```
    ros2 run speech_service synthesis
    ```
  - 新しい端末を開いて音声認識を起動します．
    ```
    ros2 run speech_service recognition
    ```
  - マイクに向かって発話すると同じ発話がスピーカから返ってきます．
  
- サービスによるオウム返しの実行手順（3.3.2節）
  - 音声サービスを起動します．
    ```
    ros2 run speech_service speech_service
    ```
  - 音声サービスに開始命令を送ります．
    ```
    ros2 service call /speech_service/wake_up airobot_interfaces/srv/StringCommand "{command: 'start'}"
    ```

## ヘルプ
- 音声認識の実行手順（3.1節）において，音声がスピーカーから出力されない場合は，synthesis_mpg123.pyからの実行を試してください．音声合成されたmp3ファイルが出力されるので，そのファイルを再生して確認してください．
- サービスによるオウム返しの実行手順（3.3.2節）において，音声がスピーカーから出力されない場合は，speech_service_mpg123.pyからの実行を試してください．音声合成されたmp3ファイルが出力されるので，そのファイルを再生して確認してください． 

## 著者
萩原　良信

## 履歴
バグフィクスやチェンジログ
- 2022-08-23: READMEの修正
- 2022-03-15: 初期版

## ライセンス
Copyright (c) 2022, Yoshinobu Hagiwara and Masaki Ito
All rights reserved.
This source code is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
- 今のところなし．
