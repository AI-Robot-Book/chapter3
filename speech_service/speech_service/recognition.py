#[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy
import rclpy.node
from std_msgs.msg import String

#[*] 音声認識を行うためのモジュールを読み込みます．
import speech_recognition as sr


#[*] Recognitionクラスをノードとして使うために，Nodeクラスを継承します．
class Recognition(rclpy.node.Node):
    def __init__(self):
        #[*] ノードの名前を'speech_recognition'として登録します．
        super().__init__('speech_recognition')

        #[*] ノードが起動したことをloggerを用いて表示します．
        self.get_logger().info('音声認識ノードを起動します')

        #[*] 音声認識器の初期化を行います．
        self.init_rec = sr.Recognizer()

        #[*] 認識した音声をトピックspeechに送るために，Publisherを初期化します．
        self.publisher = self.create_publisher(String, '/speech', 1)
        
        #[*] 5秒ごとに音声認識を行なってほしいので，Timerを初期化します．
        #[*] 実行する関数はrecognitionです．
        self.timer = self.create_timer(5.0, self.recognition)

    def recognition(self):
        #[*] 収音データを扱えるようにします．
        with sr.Microphone() as source:
            text = ''

            #[*] 収音データを5秒間分取り出せるようにします．
            audio_data = self.init_rec.record(source, duration=5)
            self.get_logger().info(f'音声認識を行います')

            try:
                #[*] Googleの音声認識器に収音データを送り，音声認識の結果を受け取ります．
                text = self.init_rec.recognize_google(audio_data)

            except sr.UnknownValueError:
                pass

            #[*] 音声認識の結果が得られた場合，speechトピックに音声認識の結果を送ります．
            if text != '':
                msg = String()
                msg.data = text
                self.get_logger().info(
                    f'認識した音声 "{text}" をトピック名 /speech にパブリッシュします')

                self.publisher.publish(msg)


def main():
    #[*] rclpyを通したrosのコミュニケーションが行えるようにします．
    rclpy.init()

    #[*] 音声認識のノードを初期化します．
    recognition_node = Recognition()
    
    try:
        #[*] 音声認識のノードを実行します．
        rclpy.spin(recognition_node)
    except KeyboardInterrupt:
        pass

    #[*] rclpyを通したrosのコミュニケーションを終了します．
    rclpy.shutdown()
