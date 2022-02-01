# ros2_voiceroid2
ROS2 wrapper package of VOICEROID2

__Windows Only__

## Installation
1. Install VOICEROID2 (x64).
1. Install Python 3.4 (x64) or later.
1. Install ROS2 foxy or later.
1. `pip install simpleaudio`
1. `pip install git+https://github.com/Nkyoku/pyvcroid2.git`
1. Clone this repository.
1. Build this repository.  
`colcon build --merge-install --packages-select voiceroid2`

## Usage
1. Source this package.  
   `./install/local_setup.ps1`
1. Run the publisher node.  
   `ros2 run voiceroid2 talker`
1. Send text to `/text` topic as `std_msgs/String`.  
   ex. `ros2 topic pub -1 /text std_msgs/String '{data: "こんにちは"}'`  
   __Text encoding must be UTF-8.__  
   In C++, UTF-8 string literal (ex. `u8"こんにちは"`) is available from C++11.

## Parameters
- `language` : string  
  Name of the language library.  
  If `language` is not specified, default value will be used.  
  Leave this parameter in ordinary use.
- `voice` : string  
  Name of the voice library.  
  If `voice` is not specified, first found one will be used.
- `subscribe_topic_name` : string  
  Topic name that the talker node subscribes.
  The message type of the topic is `std_msgs/String`.  
  Default : `text`
- `publish_topic_name` : string  
  Topic name that the talker node publishes speech data.
  The message type of the topic is `std_msgs/ByteMultiArray`.  
  If `publish_topic_name` is not specified, the speech data will be playeby local computer which the talker node runs on.
- `phrase_dictionary` : string  
  Path of the phrase dictionary.  
  Default : `<User's documents folder>/VOICEROID2/フレーズ辞書/user.pdic`
- `word_dictionary` : string  
  Path of the word dictionary.  
  Default : `<User's documents folder>/VOICEROID2/単語辞書/user.wdic`
- `symbol_dictionary` : string  
  Path of the symbol dictionary.  
  Default : `<User's documents folder>/VOICEROID2/記号ポーズ辞書/user.sdic`
- `play_mode` : string  
  Behavior when speaking multiple text.  
  - `stop` : Stop speaking previous text.
  - `wait` : Wait for finishing previous text.
  - `overlap` : Speak simultaneously.
