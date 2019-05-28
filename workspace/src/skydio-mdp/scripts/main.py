import rospy
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
import time
import sys
import os


# Abstract grid world imports.
from simple_rl.apmdp.AP_MDP.LtlAMDPClass import LTLAMDP
from simple_rl.apmdp.settings.build_cube_env_3 import build_cube_env

#seq2seq import
from simple_rl.lggltl.models.torch.experiment import text_to_ltl

WAYPOINT_TABLE = {'north': (0, 0.5, 0), 'south': (0, -0.5, 0), 'west': (-0.5, 0, 0), 'east': (0.5, 0, 0), 'up': (0, 0, 0.15), 'down': (0, 0, -0.15)}

command = ""
prev_command = ""
# hololens
landmark1_pos = (-1, -1, -1)
landmark2_pos = (-1, -1, -1)
landmark3_pos = (-1, -1, -1)
drone_pos = None
drone_path = Float32MultiArray()
cube_env = build_cube_env()

def drone_callback(data):
    """
    Assume the space is 4.5 x 4.5 x 0.6 meters, a 9 x 9 x 2 grid
    """
    global drone_pos

    z = 0
    if data.z > 0.3:
        z = 1
    # x, y, z plus 1 for amdp settings
    drone_pos = (int(data.x / 0.5) + 1, int(data.y / 0.5) + 1, z + 1)

def box_callback(data):
    global landmark1_pos
    global landmark2_pos
    global landmark3_pos

    positions = (data.data).split()
    landmark1_pos = (int(positions[0]) + 1, int(positions[1]) + 1, int(positions[2]) + 1)
    landmark2_pos = (int(positions[3]) + 1, int(positions[4]) + 1, int(positions[5]) + 1)
    landmark3_pos = (int(positions[6]) + 1, int(positions[7]) + 1, int(positions[8]) + 1)

def command_callback(data):
    print("going to callback")
    global command

    command = str(data.data)

def plan(init_loc, ltl_formula, ap_maps):
    global cube_env

    start_time = time.time()
    ltl_amdp = LTLAMDP(ltl_formula, ap_maps, env_file=[cube_env], slip_prob=0.0, verbose=True)

    sseq, aseq, len_actions, backup = ltl_amdp.solve(init_loc, FLAG_LOWEST=False)

    computing_time = time.time() - start_time

    # make the prettier output
    #s_seq, a_seq, r_seq, f_seq = ltl_amdp.format_output(sseq, aseq)

    print("Summary")
    print("\t Time: {} seconds, the number of actions: {}, backup: {}"
	  .format(round(computing_time, 3), len_actions, backup))

    path = []
    for actions in aseq:
        for action in actions:
            path.extend(list(WAYPOINT_TABLE[action]))

    print("Path: ", path)

    return path

rospy.init_node("path_pub")
pub = rospy.Publisher('/pidrone/path', Float32MultiArray, queue_size=1)
rospy.Subscriber("/pidrone/drone_position", Point, drone_callback)
rospy.Subscriber("/hololens/box_position", String, box_callback)
#rospy.Subscriber("/hololens/language_command", String, command_callback)
rospy.Subscriber("/hololens/record", String, command_callback)

time.sleep(3)

def formating(s): #s is LTL string output from seq2seq model
    global landmark1_pos
    global landmark2_pos
    global landmark3_pos
    # dictionaries
    floor_dict = {'first_floor': [2, 'state', 1], 'second_floor': [2, 'state', 2], 'third_floor': [2,'state',3]}
    color2room_dict = {'yellow_room': [1,'state',1], 'red_room': [1,'state',3], 'blue_room': [1,'state',8], 'green_room': [1,'state',11], 'purple_room': [1,'state',13], 'orange_room': [1,'state',18]}
    lm2coord_dict = {'landmark_1': [0, 'state', landmark1_pos], 'landmark_2': [0, 'state', landmark2_pos], 'landmark_3': [0, 'state', landmark3_pos]}
    print(landmark1_pos, landmark2_pos, landmark3_pos)
    ltl_formula = ''
    ap_maps = {}
    i = 0
    lst = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k']
    s = s.split()
    for c in s:
        if c in color2room_dict:
            val = color2room_dict[c]
            char = lst[i]
            ap_maps[char] = val
            c = char
            i += 1
        elif c in lm2coord_dict:
            val = lm2coord_dict[c]
            char = lst[i]
            ap_maps[char] = val
            c = char
            i += 1
        elif c in floor_dict:
            val = floor_dict[c]
            char = lst[i]
            ap_maps[char] = val
            c = char
            i += 1
        elif c == '<EOS>':
            c = ''
        else:
            pass
        ltl_formula += c + ' '
    return (ltl_formula, ap_maps)

while True:
    #if True:
    if command != prev_command:
        print("drone pos:")
        print(drone_pos)
        print("command:")
        print(command)
        prev_command = command
        
        # landmark1 = (4,1,2)
        # landmark2 = (1,2,1)
        # landmark3 = (6,3,1)
        #command = "avoid landmark two until you have been to the blue room"   #(1,1,1) -> (3,5,1)
        # command = "navigate to the red room"                                  #(3,5,1) -> (5,2,1)
        # command = "move to the orange room then the purple room"              #(5,2,1) -> (2,4,2)
        # command = "go to landmark three then go to the yellow room"           #(2,4,2) -> (2,2,1)

        #command = "go to the second floor without going to the green room"    #(2,2,1) -> ()

        #drone_pos = (1,1,1)
    
        #text to LTL
        ltl_format = 'F ( blue_room )'
        ltl = text_to_ltl(command, ltl_format)

        #format LTL formula and plan
        (ltl_formula, ap_maps) = formating(ltl[0])
        drone_path.data = plan(drone_pos, ltl_formula, ap_maps)
        
        pub.publish(drone_path)

    #drone_path.data = plan(drone_pos, 'F(a & F( b & Fc))', {'a':[1, 'state', 13], 'b':[1, 'state', 3], 'c':[0, 'state', (6, 3, 1)]})
    #ltl_formula = 'F(a & Fb)' # ex) 'F a', '~a U b', 'F(a & ~b)', 'F(a & Fb)', 'F(a & F( b & Fc))'
    #ap_maps = {'a':[2, 'state', 2]}
    #ap_maps = {'a':[0, 'state', (4, 1, 2)], 'b':[1, 'state', 11]}
    #ap_maps = {'a':[1, 'state', 8], 'b':[0, 'state', (3, 5, 1)]}
    #ap_maps = {'a':[1, 'state', 18], 'b':[1, 'state', 1]}
    #ap_maps = {'b':[0, 'state', (4, 1, 2)], 'a':[0, 'state', (3, 5, 1)]}
    #ap_maps = {'a':[1, 'state', 13], 'b':[1, 'state', 3], 'c':[0, 'state', (6, 3, 1)]}
    #drone_path.data = plan(drone_pos, ltl_formula, ap_maps)
    #pub.publish(drone_path)

    # time.sleep(1)
    # exit()


