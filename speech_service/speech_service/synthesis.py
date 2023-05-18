import rclpy #[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy.node
from std_msgs.msg import String

from gtts import gTTS #[*] gTTSはGoogleの音声合成APIを利用できるモジュールです．
from io import BytesIO #[*] 取得した音声合成データを再生するためのモジュールです．
from mpg123 import Mpg123, Out123


class Synthesis(rclpy.node.Node): #[*] Synthesisクラスをノードとして使うために，Nodeクラスを継承します．
    def __init__(self):
        super().__init__('speech_synthesis') #[*] ノードの名前を'speech_synthesis'として登録します．

        self.get_logger().info('音声合成ノードを起動します') #[*] ノードが起動したことをloggerを用いて表示します．

        self.lang = 'en' #[*] 音声合成してほしい言語の設定します．
        self.mp3 = Mpg123() #[*] 音声合成データを再生するためのクラスを初期化します．
        self.out = Out123()

        self.subscriber = self.create_subscription( #[*] 音声合成してほしい文章を取得するためのSubscriptionを初期化します．受け取った文章はsynthesis関数で実行されます．
            String, '/speech', self.synthesis, 1)

    def synthesis(self, msg):
        if msg.data != '': #[*] 音声合成してほしい文章を取得した場合
            self.get_logger().info('音声合成を実行します')

            text = msg.data
            self.get_logger().info(f'\"{text}\"と発話します')

            tts = gTTS(text, lang=self.lang[:2]) #[*] 音声合成してほしい文章をgoogleの音声合成APIに渡します．
            fp = BytesIO() #[*] 音声合成データを再生するためのIOの初期化を行います．
            tts.write_to_fp(fp)
            fp.seek(0)
            self.mp3.feed(fp.read())

            for frame in self.mp3.iter_frames(self.out.start): #[*] 音声合成データを再生します．
                self.out.play(frame)


def main():
    rclpy.init() #[*] rclpyを通したrosのコミュニケーションが行えるようにします．

    synthesis_node = Synthesis() #[*] 音声認識のノードを初期化します．

    try:
        rclpy.spin(synthesis_node) #[*] 音声認識のノードを実行します．
    except KeyboardInterrupt:
        pass

    rclpy.shutdown() #[*] rclpyを通したrosのコミュニケーションを終了します．
