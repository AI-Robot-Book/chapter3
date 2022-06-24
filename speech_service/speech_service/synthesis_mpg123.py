import rclpy
import rclpy.node
from std_msgs.msg import String

from gtts import gTTS
import subprocess


class Synthesis(rclpy.node.Node):
    def __init__(self):
        super().__init__('speech_synthesis')

        self.get_logger().info('音声合成ノードを起動します')

        self.lang = 'en'

        self.subscriber = self.create_subscription(
            String, '/speech', self.synthesis, 1)

    def synthesis(self, msg):
        self.get_logger().info('音声合成を実行します')

        text = msg.data
        self.get_logger().info(f'\"{text}\"と発話します')

        gTTS(text, lang='en', slow=True).save('voice.mp3')

        subprocess.run(['mpg123 voice.mp3'], shell=True)


def main():
    rclpy.init()

    synthesis_node = Synthesis()

    try:
        rclpy.spin(synthesis_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
