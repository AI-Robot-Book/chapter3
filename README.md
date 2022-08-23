# 第３章　音声認識・合成
## 概要
ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）第３章のサンプルプログラムと補足情報などを掲載しています．

## ディレクトリ構成

- **docker:** この本で使うROS2の開発環境，サンプルプログラムなど一式を丸ごと入れたDockerfileとDockerイメージを使うための準備作業と設定の説明です．OSはLinux (Ubuntu20.04)です．リアルロボットを動かす場合はこちらを利用してください．作成中．．．
- **docker_usage:** Dockerコンテナの使い方．作成中．．．
- **docker_windows:** この本で使うROS2の開発環境，サンプルプログラムなど一式を丸ごと入れたDockerfileとDockerイメージを使うための準備作業と設定の説明です．OSはWindowsのWSL2を使います．シミュレータのロボットを動かすだけの場合はこちらでOKです．ただし，リアルロボットを動かすことはできません．作成中．．．
- **happy_mini_open_platform:** Happy Mini Open Platformの情報を掲載します．作成中．．．
- **happy_mini_simulator:** この本で使うHappy Mini Simulatorの情報を掲載します．作成中．．．
- **non_docker:** Dockerを使わない場合のセットアップ．作成中．．．
   
## 補足情報

- 注意事項
  - 実行するUbuntuの環境でマイクからの音声入力とスピーカーからの音の出力が出来ていることを事前に確認してください．
  - バーチャルマシン上のUbuntuを使用する場合は、遅れが生じて音声が出力されない事があります．長めの発話文を入力するなどして対応して下さい．
  - 可能であればベッドセットを使用してください．オウム返しの実行中にスピーカーの音がマイクに回って繰り返す場合があります．
- 音声認識の実行手順（3.1節）
  - 端末を開いて /speech トピックへパブリッシュしたデータを見られるようにします．
    - ros2 topic echo /speech
  - 新しい端末を開いて音声認識を起動します．
    - ros2 run speech_service recognition
  - マイクに向かって発話します．
- 音声合成の実行手順（3.2節）
  - 端末を開いて音声合成を起動します．
    - ros2 run speech_service synthesis
  - 新しい端末を開いて発話させたいメッセージを /speech に送ります．
    - ros2 topic pub -1 /speech std_msgs/msg/String "{data: 'I will go to the kitchen and grab a bottle.'}"
  - スピーカから音声が出力されます．
  - 補足
    - 音声がスピーカーから出力されない場合は，synthesis_mpg123.pyからの実行を試してください．音声合成されたmp3ファイルが出力されるので，そのファイルを再生して確認してください．
- トピック通信によるオウム返しの実行手順（3.3.1節）
  - 端末を開いて音声合成を起動します．
    - ros2 run speech_service synthesis
  - 新しい端末を開いて音声認識を起動します．
    - ros2 run speech_service recognition
  - マイクに向かって発話すると同じ発話がスピーカから返ってきます．
- サービスによるオウム返しの実行手順（3.3.2節）
  - 音声サービスを起動します．
    - ros2 run speech_service speech_service
  - 音声サービスに開始命令を送ります．
    - ros2 service call /speech_service/wake_up airobot_interfaces/srv/StringCommand "{command: 'start'}"
  - 補足
    - 音声がスピーカーから出力されない場合は，speech_service_mpg123.pyからの実行を試してください．音声合成されたmp3ファイルが出力されるので，そのファイルを再生して確認してください．
