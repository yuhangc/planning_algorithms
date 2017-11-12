import numpy as np

from algorithms.sampling_based.rrt.rrt_base import RRTBase


class RRTNaive(RRTBase):
    def __init__(self, sampler, cchecker, nn, sfunc, inc=0.25):
        super(RRTNaive, self).__init__(sampler, cchecker, nn, sfunc)

        self.inc = inc
        self.nodes = []
        self.pred = []

    def solve(self, max_iter=2000, visualize=False, ax=None):
        # initialize tree with init goal
        self.nodes.append(self.init_state)
        self.nn.insert(self.init_state)
        self.pred.append(-1)

        for k in range(max_iter):
            print k
            # sample a state
            sampled_state, flag_goal = self.sampler.sample()

            # keep sampling until get a collision-free state
            while not self.cchecker.check(sampled_state):
                sampled_state, flag_goal = self.sampler.sample()

            # find nearest neighbor
            id_nn = self.nn.nn(sampled_state)
            state_nn = self.nodes[id_nn]

            # steer towards the sampled state
            if flag_goal:
                traj = self.sfunc.steer(state_nn, sampled_state, epsilon=self.inc,
                                        flag_ending=True, tol_ending=self.goal_tol)
            else:
                traj = self.sfunc.steer(state_nn, sampled_state, epsilon=self.inc)

            # continue if trajectory is empty
            if len(traj) == 0:
                continue

            # check if path is collision free
            if not self.cchecker.check_traj(traj):
                continue

            # add new node
            new_state = traj[-1]
            new_state = new_state[:3]
            self.nodes.append(new_state)
            self.nn.insert(new_state)
            self.pred.append(id_nn)

            # visualize
            ax.plot(traj[:, 0], traj[:, 1], '-', color=(0, 0, 0.8))

            # check if goal reached
            dpos = np.linalg.norm(new_state[:2] - self.goal_state[:2])
            drot = np.abs(new_state[2] - self.goal_state[2])

            if dpos < self.goal_tol[0] and drot < self.goal_tol[1]:
                break
