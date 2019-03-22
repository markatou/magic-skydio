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

- **skydio-skills**

    The skydio-skills folder contains the **com** skill. This is uploaded to R1 and instructs R1 to send its flight information to the base station. 

- **web** 

    The web folder folder contains a basic webpage to listen to information provided by R1. Note that this requires an installation of ROSBridge. 

- **workspace**

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


### Skydio -> Base Station

- On the base station command line run: 
    ```
    python magic-skydio/workspace/src/skydio-state/scripts/com_link_demo.py --skill-key com.com_link.ComLink --loop
    
    ```
    If you use a simulator, additionally include the following arguments: 
    - `--token-file` pointing to the downloaded `Simulator Auth Token` file
    - `--baseurl` equal to `Simulator URI` from the Developer Console

- On the base station command line run: 
    ``` 
    python magic-skydio/workspace/src/skydio-state/scripts/listener.py
    ```
    to display the received data.


### Base Station -> Skydio

- On the base station command line run: 
    ```
    python magic-skydio/workspace/src/skydio-state/scripts/com_link_demo.py --skill-key com.com_link.ComLink --loop
    
    ```
    If you use a simulator, additionally include the following arguments: 
    - `--token-file` pointing to the downloaded `Simulator Auth Token` file
    - `--baseurl` equal to `Simulator URI` from the Developer Console

- On the base station command line run: 
    ``` 
    python magic-skydio/workspace/src/skydio-state/scripts/publisher.py
    ```
    to send a command to R1. 
    
- Watch R1 move forward.

## (Optional) ROSBridge 

### Preliminary Networking 

Networking configuration overview: 
![Networks](https://github.com/markatou/magic-skydio/blob/ros-skydio/networks.png)

There are multiple ways to achieve the above network. We tested the following:

- Start a router with DHCP service. This is the network master. 
- Connect base station to:
    1. The network broadcasted by the master router. 
    2. The network broadcasted by the Skydio R1. 
    (Note that this requires 2 WiFi cards.) 
- Set ROS_MASTER_URI to the IP assigned by the master router to the base station. 
- Set ROS_IP to the IP assigned by the master router to the base station. 
(This can be initialized per session, or set to persist in ~/.bashrc)
- Open TCP port 9090 for inbound and outbound connections on the base station. 




### Launching 
- Open Webpage (magic-skydio/web/webpage.html) to a browser of your choice. We tested it on Chrome. 
  On the base station command line: 
  
        ```
        roscore
        roslaunch rosbridge_server rosbridge_websocket.launch
        
        ```
- View the ROS messages in the browser console.
       


