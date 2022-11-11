#[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy
import rclpy.node
from std_msgs.msg import String

#[*] gTTSはGoogleの音声合成APIを利用できるモジュールです．
from gtts import gTTS

#[*] 取得した音声合成データを再生するためのモジュールです．
from io import BytesIO
from mpg123 import Mpg123, Out123


#[*] Synthesisクラスをノードとして使うために，Nodeクラスを継承します．
class Synthesis(rclpy.node.Node):
    def __init__(self):
        #[*] ノードの名前を'speech_synthesis'として登録します．
        super().__init__('speech_synthesis')

        #[*] ノードが起動したことをloggerを用いて表示します．
        self.get_logger().info('音声合成ノードを起動します')

        #[*] 音声合成してほしい言語の設定します．
        self.lang = 'en'
        #[*] 音声合成データを再生するためのクラスを初期化します．
        self.mp3 = Mpg123()
        self.out = Out123()

        #[*] 音声合成してほしい文章を取得するためのSubscriptionを初期化します．
        #[*] 受け取った文章はsynthesis関数で実行されます．
        self.subscriber = self.create_subscription(
            String, '/speech', self.synthesis, 1)

    def synthesis(self, msg):
        #[*] 音声合成してほしい文章を取得した場合
        if msg.data != '':
            self.get_logger().info('音声合成を実行します')

            text = msg.data
            self.get_logger().info(f'\"{text}\"と発話します')

            #[*] 音声合成してほしい文章をgoogleの音声合成APIに渡します．
            tts = gTTS(text, lang=self.lang[:2])
            
            #[*] 音声合成データを再生するためのIOの初期化を行います．
            fp = BytesIO()
            tts.write_to_fp(fp)
            fp.seek(0)
            self.mp3.feed(fp.read())

            #[*] 音声合成データを再生します．
            for frame in self.mp3.iter_frames(self.out.start):
                self.out.play(frame)


def main():
    #[*] rclpyを通したrosのコミュニケーションが行えるようにします．
    rclpy.init()

    #[*] 音声認識のノードを初期化します．
    synthesis_node = Synthesis()

    try:
        #[*] 音声認識のノードを実行します．
        rclpy.spin(synthesis_node)
    except KeyboardInterrupt:
        pass

    #[*] rclpyを通したrosのコミュニケーションを終了します．
    rclpy.shutdown()
