from enum import Enum
import math


"""Legal binary comparison operations for state transitions.
"""
EQU = 0
APPR = 1
LT = 2
LTE = 3
GT = 4
GTE = 5


class SystemStateVector:
    def __init__(self):
        self.vec = {}

    def __setitem__(self, key, value):
        self.vec[key] = value

    def __getitem__(self, key):
        return self.vec[key]


class StateTransition:
    def __init__(self, lhs, comp, rhs):
        self.lhs = lhs
        self.comp = comp
        self.rhs = rhs

    def eval(self, ssv):
        lhs_v = ssv[self.lhs]
        rhs_v = ssv[self.rhs]

        if self.comp == EQU:
            return lhs_v == rhs_v
        elif self.comp == APPR:
            return math.fabs(lhs_v - rhs_v) < 0.001
        elif self.comp == LT:
            return lhs_v < rhs_v
        elif self.comp == LTE:
            return lhs_v <= rhs_v
        elif self.comp == GT:
            return lhs_v > rhs_v
        elif self.comp == GTE:
            return lhs_v >= rhs_v

        raise ValueError("unknown comparator value: %s" % comp)


class StateMachine:
    def __init__(self):
        self.states = {}
        self.transitions = {}

    def add_state(self, name, f_run):
        self.states[name] = f_run

    def add_transition(self, from, to, trans):
        if from not in transitions:
            transitions[from] = []

        transitions[from].append((to, trans))

    def run(self):
        pass
