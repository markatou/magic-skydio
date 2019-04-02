##!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from Tkinter import Tk, Label, Button, Entry, StringVar, LEFT, RIGHT, W, E # lowercase T (tkinter) for python3

class SkydioControllerGUI:
    def __init__(self, master):
        self.master = master
        master.title("Skydio Controller v0.1")

        ### Position settings
        self.positionLabel = Label(master, text="R1 Position")
        self.positionLabel.grid(row=0, sticky=W)
        self.positionLabel.config(font=("Courier", 15))

        self.currPositionLabel = Label(master, padx=10, text="[0.0, 0.0, 0.0]")
        self.currPositionLabel.grid(row=0, column=1, sticky=E)
        self.currPositionLabel.config(font=("Courier", 12))

        # x
        self.xPoseEntryLabel = Label(master, text="x: ")
        self.xPoseEntryLabel.grid(row=1)
        self.xPoseEntryLabel.config(font=("Courier", 12))

        self.xSV = StringVar()
        self.xSV.trace("w", self.enable_callback)

        self.xPoseEntry = Entry(master, textvariable=self.xSV)
        self.xPoseEntry.grid(row=1, columnspan=2)
        self.xPoseEntry.focus_set()

        # y
        self.yPoseEntryLabel = Label(master, text="y: ")
        self.yPoseEntryLabel.grid(row=2)
        self.yPoseEntryLabel.config(font=("Courier", 12))

        self.ySV = StringVar()
        self.ySV.trace("w", self.enable_callback)

        self.yPoseEntry = Entry(master, textvariable=self.ySV)
        self.yPoseEntry.grid(row=2, columnspan=2)
        self.yPoseEntry.focus_set()

        # z
        self.zPoseEntryLabel = Label(master, text="z: ")
        self.zPoseEntryLabel.grid(row=3)
        self.zPoseEntryLabel.config(font=("Courier", 12))

        self.zSV = StringVar()
        self.zSV.trace("w", self.enable_callback)

        self.zPoseEntry = Entry(master, textvariable=self.zSV)
        self.zPoseEntry.grid(row=3, columnspan=2)
        self.zPoseEntry.focus_set()

        # set position buttons
        self.setPoseButtonRel = Button(master, text="Set Position (Relative)", command=lambda: self.set_pos("r"))
        self.setPoseButtonRel.grid(row=4, padx=5, pady=5)
        self.setPoseButtonRel.config(state='disabled', font=("Courier", 12))

        self.setPoseButtonGlobal = Button(master, text="Set Position (Global)", command=lambda: self.set_pos("g"))
        self.setPoseButtonGlobal.grid(row=4, column=1, padx=5, pady=5)
        self.setPoseButtonGlobal.config(state='disabled', font=("Courier", 12))

        ### Rotation settings
        self.rotationLabel = Label(master, text="R1 Rotation")
        self.rotationLabel.grid(row=5, pady=5, sticky=W)
        self.rotationLabel.config(font=("Courier", 15))

        self.currRotationLabel = Label(master, padx=10, text="[0.0, 0.0, 0.0]")
        self.currRotationLabel.grid(row=5, column=1, sticky=E)
        self.currRotationLabel.config(font=("Courier", 12))

        # roll
        self.rollRotationEntryLabel = Label(master, text="roll: ")
        self.rollRotationEntryLabel.grid(row=6, ipadx=28, sticky=W)
        self.rollRotationEntryLabel.config(font=("Courier", 12))
        
        self.rSV = StringVar()
        self.rSV.trace("w", self.enable_callback)

        self.rollRotationEntry = Entry(master, textvariable=self.rSV)
        self.rollRotationEntry.grid(row=6, columnspan=2)
        self.rollRotationEntry.focus_set()

        # pitch
        self.pitchRotationEntryLabel = Label(master, text="pitch: ")
        self.pitchRotationEntryLabel.grid(row=7, ipadx=23, sticky=W)
        self.pitchRotationEntryLabel.config(font=("Courier", 12))
        
        self.pSV = StringVar()
        self.pSV.trace("w", self.enable_callback)

        self.pitchRotationEntry = Entry(master, textvariable=self.pSV)
        self.pitchRotationEntry.grid(row=7, columnspan=2)
        self.pitchRotationEntry.focus_set()
    
        # yaw
        self.yawRotationEntryLabel = Label(master, text="yaw: ")
        self.yawRotationEntryLabel.grid(row=8, ipadx=30, sticky=W)
        self.yawRotationEntryLabel.config(font=("Courier", 12))

        self.ywSV = StringVar()
        self.ywSV.trace("w", self.enable_callback)

        self.yawRotationEntry = Entry(master, textvariable=self.ywSV)
        self.yawRotationEntry.grid(row=8, columnspan=2)
        self.yawRotationEntry.focus_set()

        # set rotation buttons
        self.setRotationButtonRel = Button(master, text="Set Rotation (Relative)", command=lambda: self.set_rotation("r"))
        self.setRotationButtonRel.grid(row=9, padx=5, pady=5)
        self.setRotationButtonRel.config(state='disabled', font=("Courier", 12))

        self.setRotationButtonGlobal = Button(master, text="Set Rotation (Global)", command=lambda: self.set_rotation("g"))
        self.setRotationButtonGlobal.grid(row=9, column=1, padx=5, pady=5)
        self.setRotationButtonGlobal.config(state='disabled', font=("Courier", 12))

        ### Speed settings
        self.speedLabel = Label(master, text="R1 Speed")
        self.speedLabel.grid(row=10, pady=5, sticky=W)
        self.speedLabel.config(font=("Courier", 15))

        self.currSpeedLabel = Label(master, padx=10, text="0.0")
        self.currSpeedLabel.grid(row=10, column=1, sticky=E)
        self.currSpeedLabel.config(font=("Courier", 12))

        # speed
        self.speedEntryLabel = Label(master, text="speed: ")
        self.speedEntryLabel.grid(row=11)
        self.speedEntryLabel.config(font=("Courier", 12))

        self.speedEntry = Entry(master)
        self.speedEntry.grid(row=11, columnspan=2, pady=10)
        self.speedEntry.focus_set()
        self.speedEntry.config(state='disabled')

        
        ### ROS publishers 
        self.speedPub = rospy.Publisher('gui_speed_update', Float32, queue_size=10)
        self.posePub = rospy.Publisher('gui_pose_update', Pose, queue_size=10)
    
    ### Handle allowed inputs in UI
    def enable_callback(self, *args):
        xPos = self.xSV.get()
        yPos = self.ySV.get()
        zPos = self.zSV.get()
        rot = self.rSV.get()
        pitch = self.pSV.get() 
        yaw = self.ywSV.get()

        posFilled = xPos and yPos and zPos
        rpyFilled = rot and pitch and yaw 
        if posFilled:
            self.speedEntry.config(state='normal')
            self.setPoseButtonGlobal.config(state='normal')
            self.setPoseButtonRel.config(state='normal')
        if not posFilled:
            self.setPoseButtonGlobal.config(state='disabled')
            self.setPoseButtonRel.config(state='disabled')
        if rpyFilled:
            self.speedEntry.config(state='normal')
            self.setRotationButtonGlobal.config(state='normal')
            self.setRotationButtonRel.config(state='normal') 
        if not rpyFilled:
            self.setRotationButtonGlobal.config(state='disabled')
            self.setRotationButtonRel.config(state='disabled')
        if not posFilled and not rpyFilled:
            self.speedEntry.config(state='disabled')

    ### Functions to set drone state
    # Note: rel is relative to drone current state, global is relative to 0
    def set_pos(self, mode):
        global curr_pose
        try:
            x = float(self.xPoseEntry.get())
            y = float(self.yPoseEntry.get())
            z = float(self.zPoseEntry.get())
        except ValueError:
            print("Error: position input not castable to float")
            return
        time.sleep(1.0)
        if mode != "r" and mode != "g":
            print("Error: position mode not understood")
            return
        if mode == "r":
            print("setting relative pos by: %s" %('[' + str(x) + ', ' + str(y) + ', ' + str(z) + ']'))
        if mode == "g":
            print("setting global pos to: %s" %('[' + str(x) + ', ' + str(y) + ', ' + str(z) + ']'))        
        
        if self.speedEntry.get(): # include speed input
            self.set_speed()
        
        # Send ROS message
        posemsg = Pose()
        posemsg.position.x = x + (curr_pose[0][0] if mode == 'r' else 0.0)
        posemsg.position.y = y + (curr_pose[0][1] if mode == 'r' else 0.0)
        posemsg.position.z = z + (curr_pose[0][2] if mode == 'r' else 0.0)
        self.posePub.publish(posemsg)
        
        # Reset GUI
        self.xPoseEntry.delete(0, "end")
        self.yPoseEntry.delete(0, "end")
        self.zPoseEntry.delete(0, "end")

    def set_rotation(self, mode):
        global curr_pose
        try:
            r = float(self.rollRotationEntry.get())
            p = float(self.pitchRotationEntry.get())
            y = float(self.yawRotationEntry.get())
        except ValueError:
            print("Error: rotation input not castable to float")
            return
        time.sleep(1.0)
        if mode != "r" and mode != "g":
            print("Error: rotation mode not understood")
            return
        if mode == "r":
            print("setting relative rotation by: %s" %('[' + str(r) + ', ' + str(p) + ', ' + str(y) + ']'))
        if mode == "g":
            print("setting global rotation to: %s" %('[' + str(r) + ', ' + str(p) + ', ' + str(y) + ']'))        
        
        if self.speedEntry.get(): # include speed input
            self.set_speed()
        
        # Send ROS message
        posemsg = Pose()
        posemsg.orientation.x = r + (curr_pose[1][0] if mode == 'r' else 0.0)
        posemsg.orientation.y = p + (curr_pose[1][1] if mode == 'r' else 0.0)
        posemsg.orientation.z = y + (curr_pose[1][2] if mode == 'r' else 0.0)
#        posemsg.orientation.w = quaternion[3] + (curr_pose[1][3] if mode == 'r' else 0.0)
        self.posePub.publish(posemsg)
        
        # Reset GUI
        self.rollRotationEntry.delete(0, "end")
        self.pitchRotationEntry.delete(0, "end")
        self.yawRotationEntry.delete(0, "end")
    
    def set_speed(self):
        try:
            speed = float(self.speedEntry.get())
        except ValueError:
            print("Error: speed input not castable to float")
            return
        print('setting speed to: %s' %str(speed))
        speedmsg = Float32()
        speedmsg.data = speed
        self.speedPub.publish(speedmsg)
        self.speedEntry.delete(0, "end")
    
    ### Functions to get current drone state information
    def update_pose_info(self):
        global curr_pose
        if curr_pose is not None: 
            self.currPositionLabel["text"] = curr_pose[0]
            self.currRotationLabel["text"] = curr_pose[1]
        root.after(1000, self.update_pose_info) # update every 1s (no point in updating faster than http receive speed)

    def update_speed_info(self):
        global curr_speed
        if curr_speed is not None:
            self.currSpeedLabel["text"] = curr_speed
        root.after(1000, self.update_speed_info)

### Globals
global curr_pose
curr_pose = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # position, orientation 

global curr_speed
curr_speed = 0.0
### Callbacks
def pose_callback(msg):
    global curr_pose
    curr_pose[0][0] = round(msg.position.x, 3) # Rounding for simplified display
    curr_pose[0][1] = round(msg.position.y, 3)
    curr_pose[0][2] = round(msg.position.z, 3)

    curr_pose[1][0] = round(msg.orientation.x, 3) 
    curr_pose[1][1] = round(msg.orientation.y, 3)
    curr_pose[1][2] = round(msg.orientation.z, 3)
#    curr_pose[1][3] = round(msg.orientation.w, 3) #Commenting out for testing rpy

def speed_callback(msg):
    global curr_speed
    curr_speed = round(msg.data, 3)

def listener():
    ### ROS listeners
    rospy.init_node('gui', anonymous=True)
    rospy.Subscriber("pose", Pose, pose_callback)
    rospy.Subscriber("speed", Float32, speed_callback)

    ### Call GUI updates
    my_gui.update_pose_info() # Update pose
    my_gui.update_speed_info() # Update speed

### GUI
root = Tk()
my_gui = SkydioControllerGUI(root)
root.after(0, listener) # wait for publishers and subscribers
root.mainloop()