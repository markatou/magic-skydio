''' FourRoomMDPClass.py: Contains the FourRoom class. '''

# Python imports.
import math
import os
from collections import defaultdict
import numpy as np

# Other imports
from simple_rl.mdp.MDPClass import MDP
from simple_rl.apmdp.AMDP.CubeMDPClass import CubeMDP
from simple_rl.apmdp.AMDP.RoomCubeStateClass import RoomCubeState

from simple_rl.apmdp.AMDP.CubeStateClass import CubeState
from simple_rl.apmdp.settings.build_cube_env_1 import build_cube_env

from sympy import *

class RoomCubeMDP(CubeMDP):
    ''' Class for a Cube World with Rooms '''

    def __init__(self, len_x=9, len_y=9, len_z=5, init_loc=(1,1,1),
                 goal_locs=[(9,9,3)], env_file = [],
                 gamma=0.99, slip_prob=0.00, name="cube_room",
                 is_goal_terminal=True, rand_init=False,
                 step_cost=0.0, constraints={'Qg':[],'Qs':[], 'Sg': [], 'Ss': [], 'mode': 'root'}, ap_maps = {}, automata=[],
                 init_state=[]):
        '''
        Args:
            len_x, len_y, len_z (int)
            init_loc (tuple: (int, int,int))
            goal_locs (list of tuples: [(int, int,int)...]
            env_file: specify environment)
            constraints: Q_g and Q_s : goal state in automata and staty state in automata for the reward function
                        - goal (large positive), stay (zero), otherwise (large negative)
                        Sg : goal environment state, Ss: stay environment state
                        Mode: 'root' or 'child', if mode is 'root', reward is determined by automaton state
                        if mode is 'child', reward is determined by the environment state
            ap_maps: dictionary {ap_symbol: (category, state), ...} ex) {a: ('r', [1]), b:('a',west)}
                    category: floor(f), room(r), lowest level action(a), grid cells (c)
        '''

        # Load environment file

        if len(env_file)==0:
            print('Fail to initialize RoomCubeMDP')

        else:
            cube_env = env_file[0]
            len_x = cube_env['len_x']
            len_y = cube_env['len_y']
            len_z = cube_env['len_z']
            walls = cube_env['walls']
            self.num_room = cube_env['num_room']
            self.num_floor = cube_env['num_floor']
            self.room_to_locs = cube_env['room_to_locs']
            self.floor_to_rooms = cube_env['floor_to_rooms']
            self.floor_to_locs = cube_env['floor_to_locs']
            self.room_to_floor = cube_env['room_to_floor']
            self.loc_to_room = cube_env['loc_to_room']

        CubeMDP.__init__(self, len_x, len_y, len_z, init_loc,
                         goal_locs=goal_locs, walls=walls,
                         gamma=gamma, slip_prob=slip_prob, name=name,
                         is_goal_terminal=is_goal_terminal, rand_init=rand_init, step_cost=step_cost)

        self.constraints = constraints  # constraints for LTL
        self.ap_maps = ap_maps

        if self.constraints['mode'] == 'child':
            self.constraints['Ss'] = [self.get_room_numbers(init_loc)[0]]

        self.automata = automata

#        init_state = RoomCubeState(init_loc[0], init_loc[1], init_loc[2], self._transition_q(init_loc, ""))
#        if init_state.q != 0:
#            init_state.set_terminal(True)

        MDP.__init__(self, RoomCubeMDP.ACTIONS, self._transition_func, self._reward_func, init_state=init_state,
                     gamma=gamma)



    def _transition_func(self, state, action):

        if state.is_terminal():
            return state

        next_state_xyz = super()._transition_func(state, action)

        evaluated_APs = self._evaluate_APs((next_state_xyz.x, next_state_xyz.y, next_state_xyz.z), action)

        next_q = self.automata.transition_func(state.q, evaluated_APs)


        if (next_q not in self.constraints['Qg']) and (next_q not in self.constraints['Qs']):  # terminal
            next_q = -1

        next_state = RoomCubeState(next_state_xyz.x, next_state_xyz.y, next_state_xyz.z, next_q)

        next_room = self.loc_to_room[(next_state.x, next_state.y, next_state.z)]

        if self.constraints['mode'] == 'root':
            if next_state.q in self.constraints['Qg'] or next_state.q == -1:
                next_state.set_terminal(True)

        if self.constraints['mode'] == 'child':
            if next_state.q == -1 or next_state.q in self.constraints['Qg']:
                next_state.set_terminal(True)

            if next_room in self.constraints['Sg']:
                next_state.set_terminal(True)
            elif next_room not in self.constraints['Ss']:
                next_state.set_terminal(True)

        return next_state


    def is_loc_in_room(self, loc, room_number):
        return loc in self.room_to_locs[room_number]


    def is_loc_on_floor(self, loc, floor_number):
        return loc in self.floor_to_locs[floor_number]

    def get_room_numbers(self, loc):
        room_numbers = []
        for i in range(1, self.num_room+1):
            if loc in self.room_to_locs[i]:
                room_numbers.append(i)
        return room_numbers

    def get_floor_numbers(self, loc):
        room_number = self.get_room_numbers(loc)[0]
        floor_numbers = []
        for i in range(1, self.num_floor+1):
            if room_number in self.floor_to_rooms[i]:
                floor_numbers.append(i)
        return floor_numbers

    def _reward_func(self, state, action):  # TODO: Complete
        next_state = self._transition_func(state, action)
        if self.constraints['mode'] == 'root':
            if next_state.q in self.constraints['Qs']:  # stay
                reward = -1
            elif next_state.q in self.constraints['Qg']:  # success
                reward = 100
            elif next_state.q == -1:  # fail
                reward = -100
        else: # mode child
            if next_state.q == -1: # fail
                reward = -100
            elif self.loc_to_room[(next_state.x, next_state.y, next_state.z)] in self.constraints['Sg']:  # goal
                reward = 100
            else:
                reward = -1

        return reward


    def _evaluate_APs(self, loc, action): # TODO: Complete
        evaluated_APs ={}

        for ap in self.ap_maps.keys():
            if (self.ap_maps[ap][0] == 0) and (self.ap_maps[ap][1] == 'state'): # level 0
                evaluated_APs[ap] = (loc[0] == self.ap_maps[ap][2][0]) & (loc[1] == self.ap_maps[ap][2][1]) & (loc[2] == self.ap_maps[ap][2][2])

            elif (self.ap_maps[ap][0] == 0 ) and (self.ap_maps[ap][1] == 'action'):
                evaluated_APs[ap] = self.ap_maps[ap][2] in action

            elif self.ap_maps[ap][0] == 1 and (self.ap_maps[ap][1] == 'state'):  # level 1
                evaluated_APs[ap] = self.is_loc_in_room(loc, self.ap_maps[ap][2])

            elif self.ap_maps[ap][0] == 1 and (self.ap_maps[ap][1] == 'action'):  # level 1
                evaluated_APs[ap] = self.ap_maps[ap][2] in action

            elif self.ap_maps[ap][0] == 2 and (self.ap_maps[ap][1] == 'state'):  # level 2
                evaluated_APs[ap] = self.is_loc_on_floor(loc, self.ap_maps[ap][2])

            elif self.ap_maps[ap][0] == 2 and (self.ap_maps[ap][1] == 'action'):  # level 2
                evaluated_APs[ap] = self.ap_maps[ap][2] in action

        return evaluated_APs


if __name__ == '__main__':
    cube_env1 = build_cube_env()
    mdp = RoomCubeMDP(env_file=[cube_env1])
