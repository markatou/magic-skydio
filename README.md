# Magic-Skydio


This repo enables users using Skydio R1 and a base station computer to acquire R1's state through an HTTP connection, port the state to ROS, publish it as two topics (pose, speed), and then subscribe to them through a Webpage using ROSBridge. 


Use of this repo requires:  
1. Access to a base station that runs ROS melodic on Ubuntu 18.04, and rosbridge. 
2. A Skydio R1 drone.
3. A cellular device that can connect to Skydio R1. 
4. Access to Skydio SDK. 

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

