import numpy as np


class RRTBase(object):
    def __init__(self, sampler, cchecker, nn, sfunc):
        self.sampler = sampler
        self.cchecker = cchecker
        self.nn = nn
        self.sfunc = sfunc

        self.goal_state = None
        self.goal_tol = None
        self.init_state = None
        self.planned_path = None

    def set_goal(self, goal_state, goal_tol=np.array([0.1, 0.1])):
        self.goal_state = goal_state
        self.goal_tol = goal_tol

    def set_init(self, init_state):
        self.init_state = init_state

    def solve(self, max_iter=2000):
        raise Exception("Method must be implemented by derived classes!")

    def get_path(self):
        return self.planned_path
