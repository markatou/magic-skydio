# Python imports.
from __future__ import print_function

# Generic AMDP imports.
from simple_rl.amdp.AMDPSolverClass import AMDPAgent
from simple_rl.amdp.AMDPTaskNodesClass import PrimitiveAbstractTask

# Abstract grid world imports.
from simple_rl.tasks.four_room.FourRoomMDPClass import FourRoomMDP
from simple_rl.amdp.abstr_domains.grid_world.AbstractGridWorldMDPClass import FourRoomL1MDP, FourRoomL1GroundedAction, FourRoomRootGroundedAction
from simple_rl.amdp.abstr_domains.grid_world.AbstractGridWorldPolicyGeneratorClass import GridWorldL0PolicyGenerator, GridWorldL1PolicyGenerator
from simple_rl.amdp.abstr_domains.grid_world.AbstractGridWorldStateMapperClass import AbstractGridWorldL1StateMapper

# Abstract taxi imports.
from simple_rl.tasks.taxi.TaxiOOMDPClass import TaxiOOMDP
from simple_rl.amdp.abstr_domains.taxi.AbstractTaxiPolicyGeneratorClass import TaxiL0PolicyGenerator, TaxiL1PolicyGenerator
from simple_rl.amdp.abstr_domains.taxi.AbstractTaxiMDPClass import TaxiL1GroundedAction, TaxiL1OOMDP, TaxiRootGroundedAction
from simple_rl.amdp.abstr_domains.taxi.AbstractTaxiStateMapperClass import AbstractTaxiL1StateMapper

# Abstract cleanup imports.
from simple_rl.tasks.cleanup.CleanupMDPClass import CleanUpMDP
from simple_rl.amdp.abstr_domains.cleanup.AbstractCleanupMDPClass import CleanupL1MDP, CleanupL1GroundedAction, CleanupRootGroundedAction
from simple_rl.amdp.abstr_domains.cleanup.AbstractCleanupPolicyGeneratorClass import CleanupL0PolicyGenerator, CleanupL1PolicyGenerator
from simple_rl.amdp.abstr_domains.cleanup.AbstractCleanupStateMapperClass import AbstractCleanupL1StateMapper

def run_abstract_grid_world():
    start_room, goal_room = 1, 4
    l0Domain = FourRoomMDP(width=5, height=5, init_loc=(1, 1), goal_locs=[(4, 4)])
    l1Domain = FourRoomL1MDP(start_room, goal_room)

    policy_generators = []
    l0_policy_generator = GridWorldL0PolicyGenerator(l0Domain)
    l1_policy_generator = GridWorldL1PolicyGenerator(l0Domain, AbstractGridWorldL1StateMapper(l0Domain))
    policy_generators.append(l0_policy_generator)
    policy_generators.append(l1_policy_generator)

    l1Subtasks = [PrimitiveAbstractTask(action) for action in l0Domain.ACTIONS]
    a2rt = [FourRoomL1GroundedAction(a, l1Subtasks, l0Domain) for a in FourRoomL1MDP.ACTIONS]
    l1Root = FourRoomRootGroundedAction(FourRoomL1MDP.action_for_room_number(goal_room), a2rt, l1Domain,
                                        l1Domain.terminal_func, l1Domain.reward_func)

    agent = AMDPAgent(l1Root, policy_generators, l0Domain)
    agent.solve()

def run_abstract_taxi():
    agent = {"x": 1, "y": 1, "has_passenger": 0}
    passengers = [{"x": 5, "y": 1, "dest_x": 5, "dest_y": 5, "in_taxi": 0}]
    passenger = passengers[0]
    l0_domain = TaxiOOMDP(width=5, height=5, agent=agent, walls=[], passengers=passengers)

    agent_init_color = l0_domain.color_for_location((agent['x'], agent['y']))
    passenger_init_color = l0_domain.color_for_location((passenger['x'], passenger['y']))
    passenger_dest_color = l0_domain.color_for_location((passenger['dest_x'], passenger['dest_y']))
    l1_domain = TaxiL1OOMDP(agent_init_color, passenger_init_color, passenger_dest_color)

    policy_generators = []
    l0_policy_generator = TaxiL0PolicyGenerator(l0_domain)
    l1_policy_generator = TaxiL1PolicyGenerator(l0_domain, AbstractTaxiL1StateMapper(l0_domain))
    policy_generators.append(l0_policy_generator)
    policy_generators.append(l1_policy_generator)

    l1_subtasks = [PrimitiveAbstractTask(action) for action in TaxiOOMDP.ACTIONS]
    l1_tasks = [TaxiL1GroundedAction(a, l1_subtasks, l0_domain) for a in TaxiL1OOMDP.ACTIONS]

    l1_root = TaxiRootGroundedAction('ride', l1_tasks, l1_domain, l1_domain.terminal_func,
                                     l1_domain.reward_func)

    agent = AMDPAgent(l1_root, policy_generators, l0_domain)
    agent.solve()

def run_abstract_cleanup():
    def create_l0_cleanup_domain():
        from simple_rl.tasks.cleanup.cleanup_block import CleanUpBlock
        from simple_rl.tasks.cleanup.cleanup_door import CleanUpDoor
        from simple_rl.tasks.cleanup.cleanup_room import CleanUpRoom
        from simple_rl.tasks.cleanup.cleanup_task import CleanUpTask

        task = CleanUpTask("purple", "yellow")
        room1 = CleanUpRoom("room1", [(x, y) for x in range(5) for y in range(3)], "blue")
        block1 = CleanUpBlock("block1", 3, 1, color="purple")
        room2 = CleanUpRoom("room2", [(x, y) for x in range(5) for y in range(3, 6)], color="yellow")
        rooms = [room1, room2]
        blocks = [block1]
        doors = [CleanUpDoor(3, 2)]
        return CleanUpMDP(task, rooms=rooms, doors=doors, blocks=blocks)

    l0_domain = create_l0_cleanup_domain()
    l1_domain = CleanupL1MDP(l0_domain)

    l0_policy_generator = CleanupL0PolicyGenerator(l0_domain, verbose=True)
    l1_policy_generator = CleanupL1PolicyGenerator(l0_domain, AbstractCleanupL1StateMapper(l0_domain), verbose=True)
    policy_generators = [l0_policy_generator, l1_policy_generator]

    l0_init_state = l0_domain.init_state
    l1_init_state = l1_policy_generator.generate_abstract_state(l0_init_state)

    l0_actions = [PrimitiveAbstractTask(action) for action in l0_domain.ACTIONS]
    l1_actions = [CleanupL1GroundedAction(action, l0_actions, l0_domain) for action in
                  CleanupL1MDP.ground_actions(l1_init_state)]
    root_action = CleanupRootGroundedAction(str(l0_domain.task), l1_actions, l1_domain, l1_domain.terminal_func,
                                            l1_domain.reward_func)

    agent = AMDPAgent(root_action, policy_generators, l0_domain)
    agent.solve()

def main():
    print('1.\tAbstract grid world:')
    print('  \t--------------------')
    run_abstract_grid_world()
    print('\n2.\tAbstract taxi domain:')
    print('  \t--------------------')
    run_abstract_taxi()
    print('\n3.\tAbstract cleanup domain:')
    print('  \t--------------------')
    run_abstract_cleanup()

if __name__ == '__main__':
    main()
