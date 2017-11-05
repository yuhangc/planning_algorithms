import numpy as np


class RRTBase(object):
    def __init__(self, sampler, cchecker, nn, sfunc):
        self.sampler = sampler
        self.cchecker = cchecker
        self.nn = nn
        self.sfunc = sfunc

    def set_goal(self, goal_state, goal_tol):
        pass

    def set_init(self, init_state):
        pass

    def solve(self, max_iter=2000):
        pass

    def get_path(self):
        pass
