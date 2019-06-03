import rospy
import json

from std_msgs.msg import String
import time

import numpy as np

# Skydio client
from http_client import HTTPClient

# Abstract grid world imports.
from simple_rl.apmdp.AP_MDP.topology.LtlAMDPtopologyClass import LTLAMDP
from simple_rl.apmdp.settings.build_cube_env_4 import build_cube_env, draw_path

#seq2seq import
from simple_rl.lggltl.models.torch.experiment import text_to_ltl

execute = False
path = []
rooms = []
userabort = False
missionwait = False

drone_pos = (388) 


landmark1_pos = 13 #Bottom of Ruth Simmons Green
landmark2_pos = 10 # Blueno
landmark3_pos = 6 #Marcus Aurelius

locseq = []

cube_env = build_cube_env() 

WAYPOINT_TABLE = {'north': (0, 0.5, 0), 'south': (0, -0.5, 0), 'west': (-0.5, 0, 0), 'east': (0.5, 0, 0), 'up': (0, 0, 0.15), 'down': (0, 0, -0.15)}

def plan(init_loc, ltl_formula, ap_maps):
    global cube_env
    global locseq

    start_time = time.time()
    ltl_amdp = LTLAMDP(ltl_formula, ap_maps, env_file=[cube_env], slip_prob=0.0, verbose=True)

    print("solving with init loc: ", init_loc)
    sseq, aseq, len_actions, backup = ltl_amdp.solve(init_loc, FLAG_LOWEST=False)

    computing_time = time.time() - start_time

    # make the prettier output
    #s_seq, a_seq, r_seq, f_seq = ltl_amdp.format_output(sseq, aseq)

    print("Summary")
    print("\t Time: {} seconds, the number of actions: {}, backup: {}"
	  .format(round(computing_time, 3), len_actions, backup))

    path = []
    # for actions in aseq:
    #     for action in actions:
    #         print(action)
    #         #path.extend(list(WAYPOINT_TABLE[action]))
    for i in range(0, len(sseq)):
        for j in range(0, len(sseq[i])):
            id = sseq[i][j][0]
            path.append((cube_env['id_to_gps'][id][0], cube_env['id_to_gps'][id][1]))
            rooms.append(cube_env['loc_to_room'][id])

    locseq = sseq
    # print("Path: ", path)

    return path

def formating(s): #s is LTL string output from seq2seq model
    global landmark1_pos
    global landmark2_pos
    global landmark3_pos

    # dictionaries
    floor_dict = {'first_floor': [2, 'state', 1], 'second_floor': [2, 'state', 2], 'third_floor': [2,'state',3]}
    color2room_dict = {'yellow_room': [1,'state',1], 'red_room': [1,'state',3], 'blue_room': [1,'state',8], 'green_room': [1,'state',11], 'purple_room': [1,'state',13], 'orange_room': [1,'state',18]}
    lm2coord_dict = {'landmark_1': [1, 'state', landmark1_pos], 'landmark_2': [1, 'state', landmark2_pos], 'landmark_3': [1, 'state', landmark3_pos]}
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
    print("APMAPS:")
    print(ap_maps)
    return (ltl_formula, ap_maps)


def record_callback(msg):
    global execute
    global path
    global userabort
    global drone_pos

    if "abort" in msg.data:
        userabort = True
        return 

    #text to LTL
    ltl_format = 'F ( landmark_2 )'
    print("message data")
    print(msg.data)
    print("processed message data")
    ltl = text_to_ltl(msg.data, ltl_format)

    #format LTL formula and plan
    (ltl_formula, ap_maps) = formating(ltl[0])
    path = plan(drone_pos, ltl_formula, ap_maps)

    if len(path) > 0:
        execute = True


def main():
    global execute 
    global path
    global userabort
    global drone_pos
    global rooms
    global missionwait
    
    #Create the client to use for all requests.
    # client = HTTPClient('https://sim2-0.sim-us-east.skydio.com',
    #                     pilot=False,
    #                     token_file='sim2token.txt')

    client = HTTPClient('http://192.168.10.1', pilot=False)

    # Set R1 to waypoint skill
    #client.set_skill("apmdp.interiorwaypoints.LTLWaypoints")

    rospy.init_node('indoor_waypoints')
    rate = rospy.Rate(10)
    rospy.Subscriber("/pidrone/speech_final", String, record_callback)

    while not rospy.is_shutdown():
        request = {}
        if execute:
            print("RETURNED PATH:")
            print(path)

            print("AS ROOMS:")
            print(rooms)
            
            request['move'] = path
            execute = False
            missionwait = True
        if userabort:
            request['abort'] = "USER ABORT REQUESTED" # By design, skydiomain.py will now need to be killed and restarted to run another mission                

        status = client.send_custom_comms("apmdp.interiorwaypoints.LTLWaypoints", json.dumps(request).encode())

        if status == 'complete' and missionwait: #TODO: this receive of complete doesn't seem quite right
            print("rooms: ", rooms)
            print("setting drone pos room to: ", rooms[-1])
            print("setting drone locseq index to: ", locseq[0][-1][0])
            drone_pos = locseq[0][-1][0] # 294 -> 10 and 146 -> 6

            # Reset request
            path = []
            rooms = []
            execute = False
            missionwait = False

        rate.sleep()

if __name__ == "__main__":
    main()
