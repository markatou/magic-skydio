# Pidrone and Mixed Reality



## Structure of project

There are three components. The Pidrone, base station, and HoloLens. The ROS master runs on the base station.



## Pidrone

The rasberry pi is resonding for motor controlling and image capturing. Images are sent to the base station for off-board localization. <https://github.com/baichuan55555/PiDrone_Language_HoloLens>



## Base Station

The main.py connects the everything, it receives the position of the drone, the positions of landmarks, the language command in text. It will use these information to produce the path and then send it to the drone. This file should be run in python3.

The oo_search_speech_node.py used Google's speech API, converting voice to text.

Use the docker image from above link, running ```screen -c base.screenrc``` under the path of ```~/ws/src/pidrone_pkg```. Run ```roscore``` in window 0 and ```python camera_off_board_hololens.py``` under window 1 which is the localization module.



## HoloLens

This link is the unity project. Before deploy to the HoloLens, you should make sure that ROS master URL is set properly in the unity project which should be the IP of base station

https://drive.google.com/file/d/1W8f9F5pyv6hG3dvMALs7c2Zi17pl2Hqw/view?usp=sharing

