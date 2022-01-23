import rclpy
import pip
import winsound
from rclpy.node import Node
import std_msgs.msg

# Import pyvcroid2
# Install from git if not installed
try:
    import pyvcroid2
except:
    pip.main(["install", "git+https://github.com/Nkyoku/pyvcroid2.git"])
    import pyvcroid2

# Talker node
class TalkerNode(Node):
    def __init__(self):
        super().__init__('voiceroid2')

        # Initialize pyvcroid2
        self.vc = pyvcroid2.VcRoid2()
        self.vc.loadLanguage("standard")
        voice_list = self.vc.listVoices()
        if 0 < len(voice_list):
            self.vc.loadVoice(voice_list[0])
        else:
            raise Exception("No voice library")

        self.subscription = self.create_subscription(
            std_msgs.msg.String,
            'text',
            self.text_callback,
            10)

        # prevent unused variable warning
        self.subscription

    def text_callback(self, msg):
        text = msg.data
        speech, tts_events = self.vc.textToSpeech(text)
        winsound.PlaySound(speech, winsound.SND_MEMORY)

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
