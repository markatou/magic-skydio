''' GridWorldMDPClass.py: Contains the GridWorldMDP class. '''

# Python imports.
from __future__ import print_function
import random
import sys
import os
import numpy as np

# Other imports.
from simple_rl.mdp.MDPClass import MDP
from simple_rl.mdp.StateClass import State
from simple_rl.apmdp.AP_MDP.topology.GraphStateClass import GraphState

from sympy import *

# Fix input to cooperate with python 2 and 3.
try:
   input = raw_input
except NameError:
   pass

class GraphMDP(MDP):
    ''' Class for a Cube World MDP '''

    # Static constants.
    ACTIONS = ["north", "south", "west","east", "up", "down"] #""up", "down", "left", "right"]

    def __init__(self,num_node = 20,init_loc=(1), env_file=[],
                rand_init=False, goal_locs=[(5)],lava_locs=[()],walls=[],
                is_goal_terminal=True, gamma=0.99, init_state=None,
                slip_prob=0.0,step_cost=0.0,lava_cost=0.01,name="cubeworld",
                constraints={'goal': [], 'stay': []}, ap_maps={}):
        '''
        Args:
            len_x, len_y, len_z (int): the size of state space
            init_loc (tuple: (int, int, int))
            goal_locs (list of tuples: [(int, int)...])
            lava_locs (list of tuples: [(int, int)...]): These locations return -1 reward.
        '''
        self.step_cost = step_cost
        self.lava_cost = lava_cost
        self.walls = walls
        self.goal_locs = goal_locs
        self.init_state = GraphState(init_loc,0) #TODO: The 0 appended here is creating issues!
        self.is_goal_terminal = is_goal_terminal
        self.slip_prob = slip_prob
        self.name = name
        self.lava_locs = lava_locs

        if len(env_file)==0:
            print('Fail to initialize RoomCubeMDP')
        else:
            cube_env = env_file[0]
            self.num_id = cube_env['num_id']
            self.num_room = cube_env['num_room']
            self.num_floor = cube_env['num_floor']
            self.room_to_locs = cube_env['room_to_locs']
            self.floor_to_rooms = cube_env['floor_to_rooms']
            self.floor_to_locs = cube_env['floor_to_locs']
            self.room_to_floor = cube_env['room_to_floor']
            self.transition_table_l0 = cube_env['transition_table_l0']
            GraphMDP.ACTIONS = list(range(0, cube_env['num_action']))

        if 'lowest' in constraints.keys():
            self.constraints = {'goal': 'a', 'stay': 'b'}
            self.ap_maps = {'a': ap_maps['a'], 'b': [1, 'state', self.get_room_numbers(init_loc)[0]]}  # AP --> real world
            print("ap_maps: ", self.ap_maps)
        else:
            self.constraints = constraints  # constraints for LTL
            self.ap_maps = ap_maps

        init_state = GraphState(init_loc, self._transition_q(init_loc, ""))
        if init_state.q != 0:
            init_state.set_terminal(True)


        MDP.__init__(self, GraphMDP.ACTIONS, self._transition_func, self._reward_func, init_state=init_state, gamma=gamma)



    def set_slip_prob(self, slip_prob):
        self.slip_prob = slip_prob

    def get_slip_prob(self):
        return self.slip_prob

    def is_goal_state(self, state):
        return (state.x) in self.goal_locs

    def _reward_func(self, state, action): # TODO: Complete
        next_state = self._transition_func(state, action)
        #next_state = state
        if next_state.q == 0: # stay
            reward = -1
        elif next_state.q == 1:  # success
            reward = 100
        elif next_state.q == -1:  # fail
            reward = -100

        return reward

    def _is_goal_state_action(self, state, action):
        '''
        Args:
            state (State)
            action (str)

        Returns:
            (bool): True iff the state-action pair send the agent to the goal state.
        '''
        if (state.x) in self.goal_locs and self.is_goal_terminal:
            # Already at terminal.
            return False

        if action < len(self.transition_table_l0[state.x]):
            action_number = self.transition_table_l0[state.x][action]
        else:
            action_number = state.x

        if action_number in self.goal_locs:
            return True
        else:
            return False




    def _transition_func(self, state, action):
        '''
        Args:
            state (State)
            action (str)

        Returns
            (State)
        '''
        if state.is_terminal():
            return state

        r = random.random()
        next_state = state
        if self.slip_prob > r:
            # Flip dir.
            next_node = State(random.choice(self.transition_table_l0[state.x]))
            next_q = self._transition_q((next_node), action)
            next_state = GraphState(next_node, next_q)
        else:
            # print("TRANSITION TABLE: ", self.transition_table_l0)
            # print("STATE: ", state)
            # print("STATE_X: ", state.x)
            if action < len(self.transition_table_l0[state.x]):
                next_node = self.transition_table_l0[state.x][action]
                next_q = self._transition_q((next_node), action)
                next_state = GraphState(next_node, next_q)

        return next_state

    def _transition_q(self, loc, action):
        # evaluate APs
        evaluated_APs = self._evaluate_APs(loc, action)

        # q state transition
        # define symbols
        for ap in evaluated_APs.keys():
            exec('%s = symbols(\'%s\')' % (ap, ap))

        # evaluation
        if eval(self.constraints['goal']).subs(evaluated_APs):  # goal
            next_q = 1
        elif eval(self.constraints['stay']).subs(evaluated_APs):  # keep planning
            next_q = 0
        else:  # fail
            next_q = -1

        return next_q

    def _evaluate_APs(self, loc, action): # TODO: Complete
        evaluated_APs ={}
        for ap in self.ap_maps.keys():
            if self.ap_maps[ap][0] != 1 or self.ap_maps[ap][1] != 'state':
                print("ap_maps0: ", self.ap_maps[ap][0], " ap_maps1: ", self.ap_maps[ap][1])
            if (self.ap_maps[ap][0] == 0) and (self.ap_maps[ap][1] == 'state'): # level 0
                evaluated_APs[ap] = loc[0] == self.ap_maps[ap][2][0]

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

    def is_loc_in_room(self, loc, room_number):
        return loc in self.room_to_locs[room_number]


    def is_loc_on_floor(self, loc, floor_number):
        return loc in self.floor_to_locs[floor_number]

    def get_room_numbers(self, loc):
        room_numbers = []
        for i in range(0, self.num_room):
            if loc in self.room_to_locs[i]:
                room_numbers.append(i)
        return room_numbers

    def get_floor_numbers(self, loc):
        room_number = self.get_room_numbers(loc)[0]
        floor_numbers = []
        for i in range(0, self.num_floor):
            if room_number in self.floor_to_rooms[i]:
                floor_numbers.append(i)
        return floor_numbers

    def is_wall(self, x):

        return (x) in self.walls

    def __str__(self):
        return self.name + "_x-" + str(self.len_x) + "_y-" + str(self.len_y) + "_z-" + str(self.len_z)

    def __repr__(self):
        return self.__str__()

    def get_goal_locs(self):
        return self.goal_locs

    def get_lava_locs(self):
        return self.lava_locs

    def visualize_policy(self, policy):
        from simple_rl.utils import mdp_visualizer as mdpv
        from simple_rl.tasks.grid_world.grid_visualizer import _draw_state

        action_char_dict = {
            "north":"^",       #u"\u2191",
            "south":"v",     #u"\u2193",
            "west":"<",     #u"\u2190",
            "right":">",    #u"\u2192"
            "up":"+",
            "down":"-"

        }

        mdpv.visualize_policy(self, policy, _draw_state, action_char_dict)
        input("Press anything to quit")

# TODO: visualize functions
    def visualize_agent(self, agent):
        from simple_rl.utils import mdp_visualizer as mdpv
        from simple_rl.tasks.grid_world.grid_visualizer import _draw_state
        mdpv.visualize_agent(self, agent, _draw_state)
        input("Press anything to quit")

    def visualize_value(self):
        from simple_rl.utils import mdp_visualizer as mdpv
        from simple_rl.tasks.grid_world.grid_visualizer import _draw_state
        mdpv.visualize_value(self, _draw_state)
        input("Press anything to quit")

    def visualize_learning(self, agent, delay=0.0):
        from simple_rl.utils import mdp_visualizer as mdpv
        from simple_rl.tasks.grid_world.grid_visualizer import _draw_state
        mdpv.visualize_learning(self, agent, _draw_state, delay=delay)
        input("Press anything to quit")

    def visualize_interaction(self):
        from simple_rl.utils import mdp_visualizer as mdpv
        from simple_rl.tasks.grid_world.grid_visualizer import _draw_state
        mdpv.visualize_interaction(self, _draw_state)
        input("Press anything to quit")

def _error_check(state, action):
    '''
    Args:
        state (State)
        action (str)

    Summary:
        Checks to make sure the received state and action are of the right type.
    '''

    if action not in MDP.ACTIONS:
        raise ValueError("(simple_rl) CubeWorldError: the action provided (" + str(action) + ") was invalid in state: " + str(state) + ".")

    if not isinstance(state, State):
        raise ValueError("(simple_rl) CubeWorldError: the given state (" + str(state) + ") was not of the correct class.")


def main():
    grid_world = CubeMDP(5, 10,3, (1, 1,1), (6, 7,2))

    grid_world.visualize()

if __name__ == "__main__":
    main()
