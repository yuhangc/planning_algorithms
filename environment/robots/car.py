from environment.robots.robot2d_base import Robot2dRectangular
from environment.robots.robot_utils import *
from common_utils.constants import *


class BicycleCar(Robot2dRectangular):
    """
    Model of a car-like robot with bicycle model.
    """
    def __init__(self, size=(0.3, 0.5), max_v=10.0, max_ctrl=(10.0, 0.25), uncertainty=(0.0, 0.0)):
        """
        :param size: size of the car in the form of (width, length)
        :param max_v: maximum velocity of the car
        :param max_ctrl: maximum control input (max_acc, max_steer)
        :param uncertainty: acceleration and steering angle uncertainty
        """
        self.max_v = max_v
        self.max_acc, self.max_steer = max_ctrl
        self.acc_std, self.steer_std = uncertainty

        self.v = 0.0
        self.om = 0.0
        self.steer = 0.0

        super(BicycleCar, self).__init__(size)

    def update(self, u, dt):
        """
        Dynamic/kinematic update using a bicycle model.
        :param u: control input in the form (acc, steer angle)
        :param dt: time step
        """
        # sample velocity noise
        dacc = np.random.normal(0.0, self.acc_std, 1)
        dsteer = np.random.normal(0.0, self.steer_std, 1)

        # clip the control input
        u = np.clip(u + np.hstack((dacc, dsteer)),
                    np.array([-self.max_acc, -self.max_steer]), np.array([self.max_acc, self.max_steer]))

        # update velocity
        v_next = self.v + u[0] * dt
        v_next = np.clip(v_next, -self.max_v, self.max_v)

        # update position
        if np.abs(u[1]) > TOL_DYN_SIM:
            beta = np.arctan(0.5 * np.tan(u[1]))
            R = 0.5 * self.l / np.sin(beta)

            th_new = self.th + 0.5 * (self.v + v_next) * dt / R

            self.x += R * (np.sin(th_new + beta) - np.sin(self.th + beta))
            self.y -= R * (np.cos(th_new + beta) - np.cos(self.th + beta))
            self.th = th_new

            self.om = 2.0 * v_next / self.l * np.sin(beta)
        else:
            v_avg = 0.5 * (self.v + v_next)
            th_new = self.th + 0.5 * v_avg * u[1] / self.l

            self.x += v_avg * np.cos(self.th) * dt
            self.y += v_avg * np.sin(self.th) * dt
            self.th = th_new

            self.om = v_next / self.l * u[1]

        self.v = v_next
        self.steer = u[1]
