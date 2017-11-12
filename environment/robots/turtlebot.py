from environment.robots.robot2d_base import Robot2dCircular
from environment.robots.robot_utils import *
from common_utils.constants import *


class Turtlebot(Robot2dCircular):
    """
    Turtlebot is a circular differential drive robot, with velocity uncertainty
    """
    def __init__(self, size=0.3, max_vel=(0.5, 2.0), max_acc=(2.0, 5.0), uncertainty=(0.0, 0.0)):
        """
        :param size: size/radius of the robot
        :param max_vel: maximum linear and angular velocity
        :param max_acc: maximum linear and angular acceleration
        :param uncertainty: linear and angular velocity uncertainty
        """
        self.max_v, self.max_om = max_vel
        self.max_v_acc, self.max_om_acc = max_acc
        self.v_std, self.om_std = uncertainty

        self.v = 0.0
        self.om = 0.0

        super(Turtlebot, self).__init__(size)

    def update(self, u, dt):
        """
        Dynamic/kinematic update. Control inputs are commanded linear and angular velocity.
        :param u: commanded linear and angular velocities (v, om)
        :param dt: time step
        """
        # sample velocity noise
        dv = np.random.normal(0.0, self.v_std, 1)
        dom = np.random.normal(0.0, self.om_std, 1)

        v_cmd = u[0] + dv
        om_cmd = u[1] + dom

        v_next = set_cmd_vel(self.v, v_cmd, self.max_v, self.max_v_acc, dt)
        om_next = set_cmd_vel(self.om, om_cmd, self.max_om, self.max_om_acc, dt)

        # update velocity
        self.v = v_next
        self.om = om_next

        # update position
        th_new = self.th + om_next * dt

        if om_next > TOL_DYN_SIM:
            self.x += v_next / om_next * (np.sin(th_new) - np.sin(self.th))
            self.y -= v_next / om_next * (np.cos(th_new) - np.cos(self.th))
        else:
            self.x += v_next * np.cos(self.th) * dt
            self.y += v_next * np.sin(self.th) * dt

        self.th = th_new
