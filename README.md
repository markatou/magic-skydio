# ROS-Enabled Skydio

## Introduction

This project ROS-enables the Skydio R1 drone. We provide a method to access R1's pose, orientation and speed and include an example of publishing this information over ROSBridge. Lastly, we include a basic listener and example webpage for subscibing to the topics. 


## Dependencies

Use of this repo requires:  
1. [ROS melodic](http://wiki.ros.org/melodic) on [Ubuntu 18.04](https://www.ubuntu.com/download/desktop). 
2. [ROSBridge](http://wiki.ros.org/rosbridge_suite) (If using the webpage.)
2. [Skydio R1.](https://www.skydio.com/)
3. Smartphone that can connect to Skydio R1, with Skydio App installed.  
4. Access to [Skydio SDK.](https://www.skydio.com/developer/)


## Structure 

- skydio-skills

    The skydio-skills folder contains the **com** skill. This is uploaded to R1 and instructs R1 to send its flight information to the base station. 

- web 

    The web folder folder contains a basic webpage to listen to information provided by R1. Note that this requires an installation of ROSBridge. 

- workspace

    The workspace folder is a catkin workspace with one ROS package, **skydio-state**. Skydio-state contains three scripts of interest. 
    1. *com_link_demo.py* : When activated, this python script receives information from an R1 running the **com** skill, and publishes it to relevant topics. 
    2. *http_client.py* : The HTTP client contains support functions for receiving JSON responses via HTTP. 
    3. *listener.py* : A simple subscriber to the topics published by *com_link_demo.py*. 

## Networking 

[More information](https://github.com/Skydio/skydio-skills/blob/master/client/README.md)

There are two options to connect to R1, to a physical R1 or in simulation. 

### Simulation 

- Request and select a simulator in the Developer Console.  
- Click the key icon and grab the Simulator URI and download the Simulator Auth Token file to your computer. 
- In the Skydio App, under settings, select the simulator linked to your account. 

### Physical R1 

- Connect smartphone to Skydio's WiFi. 


## ROS-Enabling

Preliminaries: 
- Upload "magic-skydio/skydio-skills/com" to Skydio SDK.  
- (Optional) 
    - Open Webpage (magic-skydio/web/webpage.html) to a browser of your choice. We tested it on Chrome. 
    - On the base station command line: 
        ```
        roscore
        roslaunch rosbridge_server rosbridge_websocket.launch
        ```
### Skydio -> Base Station

- On the base station command line: 
    ```
    python magic-skydio/workspace/src/skydio-state/scripts/com_link_demo.py
    
    ```
    Arguments: 
    
    --skill-key equal to the key of the ComLink skill which is `[your_skillset_name].com_link.ComLink`
    
    --forward X Tells the Skydio to move X forward
    
    --loop 
     
    If you use a simulator: 
    - `--token-file` pointing to the downloaded `Simulator Auth Token` file
    - `--baseurl` equal to `Simulator URI` from the Developer Console

    For more information: https://github.com/Skydio/skydio-skills/blob/master/client/README.md

5. Open the Console on the Webpage, and view the ROS messages.



### Base Station -> Skydio


