import rclpy
import os
import pyvcroid2
import simpleaudio
import winsound
import threading
from ctypes import *
from rclpy.node import Node
import std_msgs.msg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult

# Talker node
class TalkerNode(Node):
    def __init__(self):
        super().__init__('voiceroid2')

        try:
            # Initialize pyvcroid2
            self.vc = pyvcroid2.VcRoid2()

            # Load language library
            language_name = self.declare_parameter("language", "standard").value
            self.vc.loadLanguage(language_name)
            self.get_logger().info("Language library '{0}' was loaded".format(language_name))

            # Load voice library
            voice_list = self.vc.listVoices()
            if len(voice_list) == 0:
                raise Exception("No voice library")
            voice_name = self.declare_parameter("voice", voice_list[0]).value
            self.vc.loadVoice(voice_name)
            self.get_logger().info("Voice library '{0}' was loaded".format(voice_name))

            # Load dictionaries
            rfid = c_char_p(b"\xD0\x9A\xD3\xFD\x8F\x23\xAF\x46\xAD\xB4\x6C\x85\x48\x03\x69\xC7")
            pwstr = c_wchar_p()
            windll.shell32.SHGetKnownFolderPath(rfid, c_uint32(0), c_void_p(), byref(pwstr))
            documents_path = wstring_at(pwstr)
            windll.ole32.CoTaskMemFree(pwstr)
            default_pdic_path = documents_path + "\\VOICEROID2\\フレーズ辞書\\user.pdic"
            default_wdic_path = documents_path + "\\VOICEROID2\\単語辞書\\user.wdic"
            default_sdic_path = documents_path + "\\VOICEROID2\\記号ポーズ辞書\\user.sdic"
            pdic_path = self.declare_parameter("phrase_dictionary", default_pdic_path).value
            wdic_path = self.declare_parameter("word_dictionary", default_wdic_path).value
            sdic_path = self.declare_parameter("symbol_dictionary", default_sdic_path).value
            if (pdic_path != default_pdic_path) or os.path.isfile(pdic_path):
                self.vc.reloadPhraseDictionary(pdic_path)
                self.get_logger().info("Phrase dictionary '{0}' was loaded".format(pdic_path))
            if (wdic_path != default_wdic_path) or os.path.isfile(wdic_path):
                self.vc.reloadWordDictionary(wdic_path)
                self.get_logger().info("Word dictionary '{0}' was loaded".format(wdic_path))
            if (sdic_path != default_sdic_path) or os.path.isfile(sdic_path):
                self.vc.reloadSymbolDictionary(sdic_path)
                self.get_logger().info("Symbol dictionary '{0}' was loaded".format(sdic_path))
        except Exception as e:
            self.get_logger().error(e)
            raise e

        # Load settings
        subscribe_topic_name = self.declare_parameter("subscribe_topic_name", "text").value
        publish_topic_name = self.declare_parameter("publish_topic_name", None).value
        play_mode = self.declare_parameter("play_mode", "stopped").value
        if play_mode == "queued":
            self.stop_before_play = False
            self.use_winsound = True
        elif play_mode == "overlapped":
            self.stop_before_play = False
            self.use_winsound = False
        else:
            self.stop_before_play = True
            self.use_winsound = False

        # Initialize node
        self.subscription = self.create_subscription(
            std_msgs.msg.String,
            subscribe_topic_name,
            self.text_callback,
            10)
        if publish_topic_name is not None:
            self.publisher = self.create_publisher(std_msgs.msg.ByteMultiArray, publish_topic_name, 10)
            self.get_logger().info("Speech data will be published as topic '{0}'".format(publish_topic_name))
        else:
            self.publisher = None
            self.get_logger().info("Speech data will be played by local computer")
        self.add_on_set_parameters_callback(self.parameter_callback)

    def text_callback(self, msg):
        text = msg.data
        if self.publisher is None:
            # Play sound in worker thread
            speech, _ = self.vc.textToSpeech(text, raw = not self.use_winsound)
            t = threading.Thread(target=self.play_sound, args=(speech,))
            t.start()
        else:
            # Publish sound data
            speech, _ = self.vc.textToSpeech(text)
            msg = std_msgs.msg.ByteMultiArray()
            msg.data = [speech]
            self.publisher.publish(msg)

    def parameter_callback(self, params):
        for param in params:
            pass
        return SetParametersResult(successful=True)

    def play_sound(self, speech):
        if self.use_winsound:
            # play_mode == "queued"
            winsound.PlaySound(speech, winsound.SND_MEMORY)
        else:
            if self.stop_before_play:
                # play_mode == "stopped"
                simpleaudio.stop_all()
            else:
                # play_mode == "overlapped"
                pass
            obj = simpleaudio.play_buffer(speech, 1, 2, 44100)
            obj.wait_done()

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
