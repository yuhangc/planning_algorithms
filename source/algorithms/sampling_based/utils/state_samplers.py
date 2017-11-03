import numpy as np


class StateSamplerBase(object):
    """
    State sampler base, with optional goal biasing
    """
    def __init__(self, goal_bias=None):
        """
        :param goal_bias: goal biasing probability. None means no goal biasing.
        """
        self.goal_bias = goal_bias
        self.goal_state = None

    def set_goal(self, goal_state):
        if self.goal_bias is None:
            raise Exception("Goal biasing mode not enabled!")
        else:
            self.goal_state = goal_state

    def sample_without_bias(self):
        raise Exception("Need to be implemented by derived classes!")

    def sample(self):
        """
        Sample with goal biasing (if enabled)
        :return: randomly sampled state
        """
        if self.goal_bias is None:
            return self.sample_without_bias()

        # obtain a random value from 0~1
        p = np.random.uniform()

        if p < self.goal_bias:
            return self.goal_state
        else:
            return self.sample_without_bias()


class StateSamplerUniform(StateSamplerBase):
    """
    Uniformly sample state from a N-dimensional rectangular region
    """
    def __init__(self, lb, ub, goal_bias=None):
        """
        :param lb: list of lower bounds
        :param ub: list of upper bounds
        """
        self.dim = len(lb)
        self.lb = lb
        self.ub = ub

        super(StateSamplerUniform, self).__init__(goal_bias)

    def sample_without_bias(self):
        state = np.zeros((self.dim, 1))

        for i in range(0, self.dim):
            state[i] = np.random.uniform(self.lb[i], self.ub[i])

        return state
