import rclpy #[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy.node
from airobot_interfaces.srv import StringCommand

import speech_recognition as sr #[*] 音声認識を行うためのモジュールを読み込みます．


from gtts import gTTS #[*] gTTSはGoogleの音声合成APIを利用できるモジュールです．


from io import BytesIO #[*] 取得した音声合成データを再生するためのモジュールです．
from mpg123 import Mpg123, Out123


class SpeechService(rclpy.node.Node): #[*] SpeechServiceクラスをノードとして使うために，Nodeクラスを継承します．
    def __init__(self):
        super().__init__('speech_service') #[*] ノードの名前を'speech_service'として登録します．
     
        self.get_logger().info('音声サーバーを起動しました') #[*] ノードが起動したことをloggerを用いて表示します．
        
        self.init_rec = sr.Recognizer() #[*] 音声認識器の初期化を行います．

        self.service = self.create_service( #[*] おうむ返しを開始するコマンドを受け取るためのServiceを作成します．リクエストが来たら，command_callback関数を呼び出します．
            StringCommand, '/speech_service/wake_up', self.command_callback)

        
        self.lang = 'en' #[*] 音声合成してほしい言語の設定します．
        self.mp3 = Mpg123() #[*] 音声合成データを再生するためのクラスを初期化します．
        self.out = Out123()
        
    def command_callback(self, request, response):

        self.synthesis("I'm ready.") #[*] サービスを開始したことを音声合成を行い，発話します．

        
        text = '' #[*] 認識した文章が受け取れるまで，音声認識を行います．
        while text == '':
            text = self.recognition()

        
        self.synthesis(text) #[*] 認識した文章を音声合成します．

        
        response.answer = text #[*] 認識結果をServiceのレンスポンスとして返します．
        return response

    def recognition(self):
        text = ''

        with sr.Microphone() as source: #[*] 収音データを扱えるようにします．
            while text == '':
                audio_data = self.init_rec.record(source, duration=5) #[*] 収音データを5秒間分取り出せるようにします．
                self.get_logger().info(f'音声認識を行います')

                try:
                    text = self.init_rec.recognize_google(audio_data) #[*] Googleの音声認識器に収音データを送り，音声認識の結果を受け取ります．
                except sr.UnknownValueError:
                    pass

        self.get_logger().info(f'認識したテキストは "{text}" です') #[*] 認識した結果をloggerで表示します．

        return text

    def synthesis(self, text):

        self.get_logger().info(f'音声合成を実行します') #[*] 音声合成を行うことをloggerで表示します．
        self.get_logger().info(f'発話内容は "{text}"')

        tts = gTTS(text, lang=self.lang[:2]) #[*] 音声合成してほしい文章をgoogleの音声合成APIに渡します．
        
        fp = BytesIO() #[*] 音声合成データを再生するためのIOの初期化を行います．
        tts.write_to_fp(fp)
        fp.seek(0)
        self.mp3.feed(fp.read())

        for frame in self.mp3.iter_frames(self.out.start): #[*] 音声合成データを再生します．
            self.out.play(frame)


def main():
    rclpy.init() #[*] rclpyを通したrosのコミュニケーションが行えるようにします．

    speech_service = SpeechService() #[*] おうむ返しのサービスを初期化します．

    try:
        rclpy.spin(speech_service) #[*] 音声認識のノードを実行します．
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown() #[*] rclpyを通したrosのコミュニケーションを終了します．
