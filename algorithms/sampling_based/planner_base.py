import numpy as np


class SamplingPlannerBase(object):
    """
    Defines the basic functionalities that a sampling based planner
    should have.
    """
    def __init__(self, collision_checker, state_sampler):
        pass

    def set_goal(self, goal_state):
        pass

    def set_init(self, init_state):
        pass

    def plan(self, max_iter):
        pass

    def get_path(self):
        pass
