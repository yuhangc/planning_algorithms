from os import path
import sys
sys.path.append(path.abspath("../../.."))

from common_utils.math import *
from common_utils.constants import *


class SteerFunctionNaive(object):
    def __init__(self, ds=0.05):
        self.ds = ds

    def steer(self, xs, xe, epsilon=None, flag_ending=False, tol_ending=None):
        # calculate position increment
        dx = xe[:2] - xs[:2]
        dx = dx / np.linalg.norm(dx) * self.ds

        traj = []
        traj.append(xs)
        s = 0.0
        x, y = xs[:2]

        if epsilon is None:
            epsilon = 1e10

        while s < epsilon:
            # update position
            x += dx[0]
            y += dx[1]
            s += self.ds

            # append to trajectory
            traj.append(np.array([x, y, 0.0]))

            # check if goal is reached
            if flag_ending and np.linalg.norm(xe[:2] - np.array([x, y])) < tol_ending[0]:
                break

        # return trajectory
        return np.array(traj)


class SteerFunctionPOSQ(object):
    def __init__(self, k_rho=1.0, k_v=3.8, k_alp=6.0, k_phi=-1.0, gamma=0.15, dt=0.05):
        """
        :param k_rho: control param
        :param k_v: control param
        :param k_alp: control param
        :param k_phi: control param
        :param gamma: distance threshold for reaching target
        :param dt: time step for forward simulation
        """
        self.k_rho = k_rho
        self.k_v = k_v
        self.k_alp = k_alp
        self.k_phi = k_phi
        self.gamma = gamma
        self.dt = dt

    def steer(self, xs, xe, epsilon=None, flag_ending=False, tol_ending=None):
        """
        :param xs: start pose
        :param xe: end pose
        :param epsilon: steering increment (optional)
        :param flag_ending: whether this is steering to the goal
        :param tol_ending: tolerance for reaching the final goal
        :return: trajectory that connects xs and xe
        """
        # initialize
        x, y, th = xs
        s = 0.0
        traj = []
        traj.append(xs)

        if epsilon is None:
            epsilon = 1e10

        while s < epsilon:
            # calculate geometric properties
            rho = np.linalg.norm(xe[:2] - np.array([x, y]))
            phi = wrap_to_pi(xe[2] - th)
            th_z = np.arctan2(xe[1] - y, xe[0] - x)
            alpha = wrap_to_pi(th_z - th)

            # check if goal is reached
            if flag_ending:
                if rho < tol_ending[0] and np.abs(phi) < tol_ending[1]:
                    break
            else:
                if rho < self.gamma:
                    break

            # calculate control
            v = self.k_rho * np.tanh(self.k_v * rho)
            om = self.k_alp * alpha + self.k_phi * phi

            # steer forward
            th_new = th + om * self.dt

            if om > TOL_DYN_SIM:
                x += v / om * (np.sin(th_new) - np.sin(th))
                y -= v / om * (np.cos(th_new) - np.cos(th))
            else:
                x += v * np.cos(th) * self.dt
                y += v * np.sin(th) * self.dt

            th = th_new
            s += v * self.dt

            # append to trajectory
            traj.append(np.array([x, y, th]))

        # return the trajectory
        if len(traj) == 1:
            return np.array([])
        else:
            return np.array(traj)
