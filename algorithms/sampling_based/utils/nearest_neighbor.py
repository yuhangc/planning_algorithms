import numpy as np


class NearestNeighborBase(object):
    def __init__(self, dist_func):
        self.dist_func = dist_func

    def insert(self, state):
        raise Exception("Method must be implemented by derived classes")

    def nn(self, state):
        raise Exception("Method must be implemented by derived classes")


class NNBruteForce(NearestNeighborBase):
    def __init__(self, dist_func):
        super(NNBruteForce, self).__init__(dist_func)
        self.node_list = []

    def insert(self, state):
        self.node_list.append(state)

    def nn(self, state):
        """
        Find nearest neighbor by brute force
        :param state: query state
        :return: Index of the nearest neighbor
        """
        nn_candidate = None
        dist_min = 1e10

        for k in range(len(self.node_list)):
            node_state = self.node_list[k]
            dist = self.dist_func(node_state, state)
            if dist < dist_min:
                dist_min = dist
                nn_candidate = k

        return nn_candidate
