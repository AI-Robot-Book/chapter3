import rclpy #[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy.node
from std_msgs.msg import String

import speech_recognition as sr #[*] 音声認識を行うためのモジュールを読み込みます．


class Recognition(rclpy.node.Node): #[*] Recognitionクラスをノードとして使うために，Nodeクラスを継承します．
    def __init__(self):
        super().__init__('speech_recognition') #[*] ノードの名前を'speech_recognition'として登録します．

        self.get_logger().info('音声認識ノードを起動します') #[*] ノードが起動したことをloggerを用いて表示します．

        self.init_rec = sr.Recognizer() #[*] 音声認識器の初期化を行います．

        self.publisher = self.create_publisher(String, '/speech', 1) #[*] 認識した音声をトピックspeechに送るために，Publisherを初期化します．

        self.timer = self.create_timer(5.0, self.recognition) #[*] 5秒ごとに音声認識を行なってほしいので，Timerを初期化します．実行する関数はrecognitionです．

    def recognition(self):
        with sr.Microphone() as source: #[*] 収音データを扱えるようにします．
            text = ''

            audio_data = self.init_rec.record(source, duration=5) #[*] 収音データを5秒間分取り出せるようにします．
            self.get_logger().info(f'音声認識を行います')

            try:
                text = self.init_rec.recognize_google(audio_data) #[*] Googleの音声認識器に収音データを送り，音声認識の結果を受け取ります．

            except sr.UnknownValueError:
                pass

            if text != '': #[*] 音声認識の結果が得られた場合，speechトピックに音声認識の結果を送ります．
                msg = String()
                msg.data = text
                self.get_logger().info(
                    f'認識した音声 "{text}" をトピック名 /speech にパブリッシュします')

                self.publisher.publish(msg)


def main():
    rclpy.init() #[*] rclpyを通したrosのコミュニケーションが行えるようにします．

    recognition_node = Recognition() #[*] 音声認識のノードを初期化します．
    
    try:
        rclpy.spin(recognition_node) #[*] 音声認識のノードを実行します．
    except KeyboardInterrupt:
        pass

    rclpy.shutdown() #[*] rclpyを通したrosのコミュニケーションを終了します．
