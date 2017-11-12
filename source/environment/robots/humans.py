from source.environment.robots.robot2d_base import Robot2dCircular
from source.environment.robots.robot_utils import *
from source.common_utils.constants import *


class HumanSimple(Robot2dCircular):
    def __init__(self, size=0.3, max_vel=2.0, max_acc=5.0, uncertainty=0.0):
        super(HumanSimple, self).__init__(size)

        self.max_v = max_vel
        self.max_acc = max_acc
        self.v_std = uncertainty

        self.vx = 0.0
        self.vy = 0.0

        self.x_goal = None
        self.vd = None
        self.k = None

        # a damping factor
        self.c = 0.1

    def set_goal(self, x_goal, vd, k):
        self.x_goal = x_goal
        self.vd = vd
        self.k = k

    def update(self, u, dt):
        """
        Dynamic update, input is the "social force" that drives people
        :param u: (fx, fy)
        :param dt: time step
        """

        # calculate the force that drives people to goal
        x_rel_goal = self.x_goal - np.array([self.x, self.y])
        v_goal = self.vd * x_rel_goal / np.linalg.norm(x_rel_goal)

        f_goal_x = self.k * (v_goal[0] - self.vx)
        f_goal_y = self.k * (v_goal[1] - self.vy)

        # calculate accelerations
        ax = f_goal_x + u[0] - self.c * self.vx
        ay = f_goal_y + u[1] - self.c * self.vy

        # update velocity and positions
        vx_next = self.vx + ax * dt
        vy_next = self.vy + ay * dt

        v_next = np.linalg.norm(np.array([vx_next, vy_next]))
        if v_next > self.max_v:
            vx_next *= self.max_v / v_next
            vy_next *= self.max_v / v_next

        self.x += 0.5 * (self.vx + vx_next) * dt
        self.y += 0.5 * (self.vy + vy_next) * dt

        self.vx = vx_next
        self.vy = vy_next
