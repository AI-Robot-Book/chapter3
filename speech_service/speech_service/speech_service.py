#[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy
import rclpy.node
from airobot_interfaces.srv import StringCommand

#[*] 音声認識を行うためのモジュールを読み込みます．
import speech_recognition as sr

#[*] gTTSはGoogleの音声合成APIを利用できるモジュールです．
from gtts import gTTS

#[*] 取得した音声合成データを再生するためのモジュールです．
from io import BytesIO
from mpg123 import Mpg123, Out123


#[*] SpeechServiceクラスをノードとして使うために，Nodeクラスを継承します．
class SpeechService(rclpy.node.Node):
    def __init__(self):
        #[*] ノードの名前を'speech_service'として登録します．
        super().__init__('speech_service')

        #[*] ノードが起動したことをloggerを用いて表示します．
        self.get_logger().info('音声サーバーを起動しました')

        #[*] 音声認識器の初期化を行います．
        self.init_rec = sr.Recognizer()

        #[*] おうむ返しを開始するコマンドを受け取るためのServiceを作成します．
        #[*] リクエストが来たら，command_callback関数を呼び出します．
        self.service = self.create_service(
            StringCommand, '/speech_service/wake_up', self.command_callback)

        #[*] 音声合成してほしい言語の設定します．
        self.lang = 'en'
        #[*] 音声合成データを再生するためのクラスを初期化します．
        self.mp3 = Mpg123()
        self.out = Out123()
        
    def command_callback(self, request, response):

        #[*] サービスを開始したことを音声合成を行い，発話します．
        self.synthesis("I'm ready.")

        #[*] 認識した文章が受け取れるまで，音声認識を行います．
        text = ''
        while text == '':
            text = self.recognition()

        #[*] 認識した文章を音声合成します．
        self.synthesis(text)

        #[*] 認識結果をServiceのレンスポンスとして返します．
        response.answer = text
        return response

    def recognition(self):
        text = ''

        #[*] 収音データを扱えるようにします．
        with sr.Microphone() as source:
            while text == '':
                #[*] 収音データを5秒間分取り出せるようにします．
                audio_data = self.init_rec.record(source, duration=5)
                self.get_logger().info(f'音声認識を行います')

                try:
                    #[*] Googleの音声認識器に収音データを送り，音声認識の結果を受け取ります．
                    text = self.init_rec.recognize_google(audio_data)
                except sr.UnknownValueError:
                    pass

        #[*] 認識した結果をloggerで表示します．
        self.get_logger().info(f'認識したテキストは "{text}" です')

        return text

    def synthesis(self, text):

        #[*] 音声合成を行うことをloggerで表示します．
        self.get_logger().info(f'音声合成を実行します')
        self.get_logger().info(f'発話内容は "{text}"')

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

    #[*] おうむ返しのサービスを初期化します．
    speech_service = SpeechService()

    try:
        #[*] 音声認識のノードを実行します．
        rclpy.spin(speech_service)
    except KeyboardInterrupt:
        pass

    #[*] rclpyを通したrosのコミュニケーションを終了します．
    rclpy.shutdown()
