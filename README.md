# ROS-Enabled Skydio

## Introduction

This project ROS-enables the Skydio R1 drone. We provide a method to access R1's pose, orientation and speed and include an example of publishing this information over ROSBridge. Lastly, we include a basic listener and example webpage for subscibing to the topics. 


## Dependencies

Use of this repo requires:  
1. [ROS melodic](http://wiki.ros.org/melodic) on [Ubuntu 18.04](https://www.ubuntu.com/download/desktop). 
2. [ROSBridge](http://wiki.ros.org/rosbridge_suite) (If using the webpage.)
2. [Skydio R1.](https://www.skydio.com/)
3. Smartphone that can connect to Skydio R1. 
4. Access to [Skydio SDK.](https://www.skydio.com/developer/)


## Structure 

This repo is structured as follows: 
- 

## ROS-Enabling

Instructions: 

1. Upload "magic-skydio/skydio-skills/com" to Skydio SDK. (Instructions: https://console.skydio.com/docs/skills/getting_started.html ) 
2. Open Webpage (magic-skydio/web/webpage.html) to a browser of your choice. We tried it on Chrome. 
3. On a command line: 

    i. roscore
    
    ii. roslaunch rosbridge_server rosbridge_websocket.launch
    
4. In a new command window, run the "magic-skydio/workspace/src/skydio-state/scripts/com_link_demo.py" 

    Arguments: 
    
    --skill-key equal to the key of the ComLink skill which is `[your_skillset_name].com_link.ComLink`
    
    --forward X Tells the Skydio to move X forward
    
    --loop 
     
    If you use a simulator: 
    - `--token-file` pointing to the downloaded `Simulator Auth Token` file
    - `--baseurl` equal to `Simulator URI` from the Developer Console

    For more information: https://github.com/Skydio/skydio-skills/blob/master/client/README.md

5. Open the Console on the Webpage, and view the ROS messages.

