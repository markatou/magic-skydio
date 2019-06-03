"""
Environment for Skydio in outdoor
"""

import numpy as np
from itertools import product
from collections import defaultdict
import geopy.distance
import matplotlib.pyplot as plt

def build_cube_env():
    # Large
    cube_env = {} # Define settings as a dictionary
    cube_env['num_node'] = 40  # the number of grids (x-axis)
    cube_env['num_floor'] = 1 # the number of floors

    num_x = 3
    num_y = 3
    room_height, cube_env['room_height'] = 1, 1
    room_len, cube_env['room_len'] = 3, 3
    num_rooms_on_floor = num_x*num_y

    # Define a map : save a room number of each cell, w (wall)
    room_up_down = [2, 3, 6, 11, 12, 15] # Drone can move up and down through these rooms


    # ----------- Room to GPS ------------- #
    room_to_gps={}
    num_room = 0
    # region 0
    lat = np.linspace(41.826381,41.826353, 11)
    long = np.linspace(-71.403525,-71.402958, 11)
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 1
    lat = np.linspace(41.826319, 41.826071, 7)
    long = np.linspace(-71.403509, -71.403030, 10)
    room_to_gps[num_room] = list(product(lat, long))
    num_room = num_room + 1
    # region 2
    lat = np.linspace(41.825943, 41.826021, 12)
    long = np.linspace(-71.403525,  -71.402900, 12)
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 3
    lat = np.linspace(41.826698, 41.825965, 18)
    long = np.linspace(-71.402976, -71.402868, 18)
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 4
    lat = np.linspace(41.826707, 41.826657, 3)
    long = np.linspace(-71.402937, -71.402791, 4)
    room_to_gps[num_room] = list(product(lat, long))
    num_room = num_room + 1
    # region 5
    lat = np.linspace(41.826349, 41.826391, 12)
    long = np.linspace(-71.402880, -71.402236, 12)
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 6
#    lat = np.linspace(41.826024,41.826089, 12)
    lat = np.linspace(41.826024, 41.826195, 12) # Near horse lat
#    long = np.linspace( -71.402867, -71.402211,12 )
    long = np.linspace( -71.402867, -71.40207, 12 ) # Near horse long
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 7
    lat = np.linspace(41.826403, 41.826456, 16)
    long = np.linspace( -71.402215,-71.401383, 16 )
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 8
    lat = np.linspace(41.826382,41.826225,6 )
    long = np.linspace(-71.402174,-71.401387,16 )
    room_to_gps[num_room] = list(product(lat, long))
    num_room = num_room + 1
    # region 9
    #lat = np.linspace(41.826090,41.826172,16 )
    lat = np.linspace(41.826170,41.826272,16 ) #Adjust into green (take 41.8261[7]0 -> 41.8261[5]0 to be further away from horse
    long = np.linspace( -71.402178, -71.401346,16)
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 10
    lat = np.linspace(41.826610,41.826499,4 )
    long = np.linspace(-71.401611,-71.401427, 5)
    room_to_gps[num_room] = list(product(lat, long))
    num_room = num_room + 1
    # region 11
    lat = np.linspace(41.826744,41.826206, 14)
    #long = np.linspace(-71.401414, -71.401331, 14 )
    long = np.linspace(-71.401380, -71.401331, 14) # Bring to other side of walkway
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 12
    lat = np.concatenate((np.linspace(41.826462,41.826474,6 ),np.linspace(41.826474,41.826380,6)[1:]))
    long = np.concatenate((np.linspace(-71.401347,-71.401052,6 ),np.linspace(-71.401052,-71.400839,6)[1:]))
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 13
    lat = np.linspace(41.826380,41.826295,6 )
    long = np.linspace(-71.401288,-71.400935, 7)
    room_to_gps[num_room] = list(product(lat, long))
    num_room = num_room + 1
    # region 14
    lat = np.concatenate((np.linspace(41.826175,41.826214,7 ), np.linspace(41.826214,41.826341,7 )[1:]))
    long = np.concatenate((np.linspace(-71.401302,-71.400966,7 ), np.linspace(-71.400966, -71.400829,7 )[1:]))
    room_to_gps[num_room] = [x for x in zip(lat, long)]
    num_room = num_room + 1
    # region 15
    lat = np.linspace(41.826320,41.826380, 6)
    long = np.linspace(-71.400935,-71.400635, 6)
    room_to_gps[num_room] = list(product(lat, long))
    num_room = num_room + 1
    # region 16
    lat = np.linspace(41.826390,41.826320, 6)
    long = np.linspace(-71.400635, -71.400600, 6)
#    room_to_gps[num_room] = [x for x in zip(lat, long)]
    room_to_gps[num_room] = list(product(lat, long))
    num_room = num_room + 1
    # region 18
#    lat = np.linspace(,, )
#    long = np.linspace(,, )
#    room_to_gps[] = [x for x in zip(lat, long)]

    # region
#    lat = np.linspace(,,)
#    long = np.linspace(,,)
#    room_to_gps[] = [x for x in zip(lat, long)]
#    room_to_gps[] = list(product(lat, long))

    cube_env['num_room'] = num_room  # the number of rooms
    # --------------------- Automatically computed --------- #
    # id to gps, id to room, room to id
    num_id = 0
    room_to_locs = defaultdict()
    loc_to_room = defaultdict()
    id_to_gps = defaultdict()
    for r in range(0, num_room):
        num_id_in_room = len(room_to_gps[r])
        for ii in range(0,num_id_in_room):
            id_to_gps[num_id+ii] = room_to_gps[r][ii]
            loc_to_room[num_id+ii] = r

        ids = list(range(num_id, num_id+num_id_in_room))
        room_to_locs[r] = ids
        num_id = num_id + num_id_in_room

    cube_env['id_to_gps'] = id_to_gps
    cube_env['room_to_locs'] = room_to_locs
    cube_env['loc_to_room'] = loc_to_room
    cube_env['num_id'] = num_id


    # compute the transition table
    transition_table = defaultdict()
    for ii in range(0, cube_env['num_id']):
        actions = []
        for jj in range(0, cube_env['num_id']):
            if gps_to_meters(cube_env['id_to_gps'][ii],cube_env['id_to_gps'][jj]) < 5.0:
                actions.append(jj)
        transition_table[ii] = actions
    cube_env['transition_table_l0'] = transition_table
    cube_env['num_action']=max([len(x) for x in transition_table.values()])

    # extract (x,y,z) in walls
    walls = []

    cube_env['walls'] = walls  # the list of ids which are walls

    # Extract room numbers and locations in each floor
    floor_to_room = defaultdict()   # {floor: the list of rooms on the floor}
    floor_to_locs = defaultdict()   # {floor: the list of cells}
    room_to_floor = {}              # {room: the corresponding floor}
    loc_to_floor ={}                # {cell: the corresponding floor}



    cube_env['floor_to_rooms'] = {0:list(range(0,cube_env['num_room']))}
    cube_env['floor_to_locs'] = {0:list(range(0,cube_env['num_id']))}
    cube_env['room_to_floor'] = dict.fromkeys(range(0,cube_env['num_room']),0)
    cube_env['loc_to_floor']= dict.fromkeys(range(0,cube_env['num_id']),0)

    # Define transition table (connectivity between rooms)
    cube_env['transition_table'] = {0:[0,1,3], 1:[0,1, 3], 2: [2,1,3], 3: [0,1,2,3,4,5,6], 4:[4,5], 5:[3,5,7],
            6:[3,6,9], 7:[5,7,8,11], 8:[7,8,9,11], 9:[6,8,9,11], 10:[10,11],
                                    11:[7,8,9,10,11,12,13,14], 12:[11,12,13,15], 13:[11,12,13,14,15],
                                    14:[11,13,14,15],15:[12,13,14], 16: [15,16]}   # dictionary {room i: rooms connected with the room i}


    # Define attributes
    cube_env['attribute_color'] = {1: 'red', 6: 'blue', 12: 'blue', 18: 'blue',
                                   8: 'yellow', 10: 'purple', 15: 'green'
                                   }

    # Define Actions
    cube_env['L2ACTIONS'] = ["toFloor%d" % ii for ii in range(0, cube_env['num_floor'])]
    cube_env['L1ACTIONS'] = ["toRoom%d" % ii for ii in range(0, cube_env['num_room'])]
    cube_env['L0ACTIONS'] = ["Goto%d" % ii for ii in range(0, cube_env['num_action'] )]

    # save
    #np.save('cube_env_1.npy',cube_env)

    return cube_env

def gps_to_meters(gps1, gps2):
    return geopy.distance.vincenty(gps1,gps2).m

def draw_graph(cube_env):
    cset =['g','r','c','m','k']
    fig, ax = plt.subplots()
    for ii in range(0,cube_env['num_id']):
        ax.scatter(cube_env['id_to_gps'][ii][0],cube_env['id_to_gps'][ii][1], c=cset[int(np.mod(cube_env['loc_to_room'][ii],5))])
        for jj in cube_env['transition_table_l0'][ii]:
            plt.plot((cube_env['id_to_gps'][ii][0],cube_env['id_to_gps'][jj][0]),
                     (cube_env['id_to_gps'][ii][1],cube_env['id_to_gps'][jj][1]), c='blue')


    plt.show()


def draw_path(sseq, cube_env):
    lat = []
    long = []
    for i in range(0,len(sseq)):
        for j in range(0,len(sseq[i])):
            id = sseq[i][j][0]
            lat.append(cube_env['id_to_gps'][id][0])
            long.append(cube_env['id_to_gps'][id][1])
    plt.plot(lat,long)

    plt.show()




if __name__ == '__main__':
    env = build_cube_env()
    draw_graph(env)
    print("done")
