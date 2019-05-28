''' GridWorldStateClass.py: Contains the GridWorldState class. '''

# Other imports.
from simple_rl.mdp.StateClass import State

class GraphState(State):
    ''' Class for Grid World States '''

    def __init__(self, x, q):
        State.__init__(self, data=[x, q])
        self.x = round(x, 5)
        self.q = round(q, 5)


    def __hash__(self):
        return hash(tuple(self.data))

    def __str__(self):
        return "s: (" + str(self.x) + "," + str(self.q) +")"

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return isinstance(other, State) and self.x == other.x and self.q == other.q
