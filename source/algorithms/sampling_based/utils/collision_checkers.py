import numpy as np

from source.environment.worlds.occupancy_grid import OccupancyGrid
from source.algorithms.sampling_based.utils.utils import *


class CollisionCheckerBase(object):
    def __init__(self, ss_map):
        """
        :param ss_map: state space map that specifies obstacles
        """
        self.ss_map = ss_map

    def check(self, state):
        """
        :param state: robot state for check
        :return: True for no collision, False for collision
        """
        raise Exception("Needs to be implemented by derived classes!")


class CollisionCheckerGrid(CollisionCheckerBase):
    """
    Collision checker for 2D occupancy grid map.
    """
    def __init__(self, ss_map, robot):
        super(CollisionCheckerGrid, self).__init__(ss_map)

        self.robot = robot
        if self.robot.type == "Circular2D":
            self.check_points = self.init_check_points_circular(self.robot.radius)
        elif self.robot.type == "Rectangular2D":
            self.check_points = self.init_check_points_rectangular(self.robot.w, self.robot.l)

    def init_check_points_circular(self, radius):
        """
        Create a set of check points in the robot's coordinate frame.
        :param radius: robot's radius
        :return: the check points
        """
        check_points = []

        # compute a bounding box first
        delta = int(np.ceil(radius / self.ss_map.resolution))

        # check all the points in bounding box
        for xx in range(-delta, delta + 1):
            for yy in range(-delta, delta + 1):
                x = xx * self.ss_map.resolution
                y = yy * self.ss_map.resolution

                if (np.sqrt(x**2 + y**2) - self.robot.radius) < 0.5 * self.ss_map.resolution:
                    check_points.append((x, y))

        return check_points

    def init_check_points_rectangular(self, w, h):
        check_points = []

        grid_w = int(round(w / self.ss_map.resolution / 2.0))
        grid_h = int(round(h / self.ss_map.resolution / 2.0))

        for xx in range(-grid_h, grid_h + 1):
            for yy in range(-grid_w, grid_w + 1):
                x = xx * self.ss_map.resolution
                y = yy * self.ss_map.resolution
                check_points.append((x, y))

        return check_points

    def check_circular(self, state):
        for point in self.check_points:
            # if ax is not None:
            #     ax.scatter(point[0] + state[0], point[1] + state[1], color="red")
            if not self.ss_map.is_free(point[0] + state[0], point[1] + state[1]):
                return False

        return True

    def check_rectangular(self, state):
        for point in self.check_points:
            # first transform the check points in world coordinate
            x, y = transform_point_to_world(point, state)
            # if ax is not None:
            #     ax.scatter(x, y, color="red")
            if not self.ss_map.is_free(x, y):
                return False

        return True

    def check(self, state, ax=None):
        if self.robot.type == "Circular2D":
            return self.check_circular(state)
        elif self.robot.type == "Rectangular2D":
            return self.check_rectangular(state)
