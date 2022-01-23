# ros2_voiceroid2
ROS2 wrapper package of VOICEROID2

__Windows Only__

## Installation
1. Install Visual Studio 2019 or later.
1. Install ROS2 foxy or later.
1. Clone this repository.
1. Build this repository.  
`colcon build --merge-install --packages-select voiceroid2`

## Usage
1. Source this package.  
   `./install/local_setup.ps1`
1. Run the publisher node.  
   `ros2 run voiceroid2 talker`
1. There are several parameters.
   - language : string  
   Name of the language library.
   If `language` is not specified, default value will be used.
   - voice : string  
   Name of the voice library.  
   If `voice` is not specified, first found one will be used.
   - topic_name : string  
   Topic name that the talker node subscribes.
