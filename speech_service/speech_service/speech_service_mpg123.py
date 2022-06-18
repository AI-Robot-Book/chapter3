import rclpy
import rclpy.node
from airobot_interfaces.srv import StringCommand

from gtts import gTTS

import speech_recognition as sr
import pyaudio
import subprocess


class SpeechService(rclpy.node.Node):
    def __init__(self):
        super().__init__("speech_service")

        self.get_logger().info("音声サーバーを起動しました")

        self.init_rec = sr.Recognizer()

        self.service = self.create_service(StringCommand, '/speech_service/wake_up', self.command_callback)

    def command_callback(self, request, response):

        self.synthesis('I\'m ready.')

        text = None
        while text is None:
            text = self.recognition()

        self.synthesis(text)

        response.answer = 'done'
        return response

    def recognition(self):
        text = ''

        with sr.Microphone() as source:
            while text == '':
                audio_data = self.init_rec.record(source, duration=5)
                self.get_logger().info(f'音声認識を行います')

                try:
                    text = self.init_rec.recognize_google(audio_data)
                except sr.UnknownValueError:
                    pass

        self.get_logger().info(f'認識したテキストは "{text}" です')

        return text

    def synthesis(self, text):

        self.get_logger().info(f'音声合成を実行します')
        self.get_logger().info(f'発話内容は "{text}"')

        gTTS(text, lang='en').save('voice.mp3')

        subprocess.run(["mpg123 voice.mp3"], shell=True)

def main():
    rclpy.init()

    speech_service = SpeechService()

    rclpy.spin(speech_service)
    speech_service.destroy_node()

    rclpy.shutdown()
