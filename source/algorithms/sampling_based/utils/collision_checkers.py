from source.environment.worlds.occupancy_grid import OccupancyGrid
from source.common_utils.math import *


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

        # boundary points
        self.boundary = self.find_boundary_points(self.check_points)

    @staticmethod
    def find_boundary_points(check_points):
        boundary = []

        # loop through all points
        N = len(check_points)
        for i in range(N):
            # initialize quadrant counter
            qcount = np.zeros((4, 1), dtype=int)
            pi = check_points[i]

            # check all other points
            for j in range(N):
                if i == j:
                    continue

                pj = check_points[j]
                if pj[0] > pi[0] and pj[1] >= pi[1]:
                    qcount[0] += 1
                if pj[0] <= pi[0] and pj[1] > pi[1]:
                    qcount[1] += 1
                if pj[0] < pi[0] and pj[1] <= pi[1]:
                    qcount[2] += 1
                if pj[0] >= pi[0] and pj[1] < pi[1]:
                    qcount[3] += 1

            # check quadrant counts
            if any(qcount == 0):
                boundary.append(pi)

        return boundary

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

    def check_circular(self, state, check_points):
        for point in check_points:
            # if ax is not None:
            #     ax.scatter(point[0] + state[0], point[1] + state[1], color="red")
            if not self.ss_map.is_free(point[0] + state[0], point[1] + state[1]):
                return False

        return True

    def check_rectangular(self, state, check_points):
        for point in check_points:
            # first transform the check points in world coordinate
            x, y = transform_point_to_world(point, state)
            # if ax is not None:
            #     ax.scatter(x, y, color="red")
            if not self.ss_map.is_free(x, y):
                return False

        return True

    def check(self, state, ax=None):
        """
        :return: True if collision free, False otherwise
        """
        if self.robot.type == "Circular2D":
            return self.check_circular(state, self.check_points)
        elif self.robot.type == "Rectangular2D":
            return self.check_rectangular(state, self.check_points)

    def check_traj(self, traj):
        """
        :return: True if collision free, False otherwise
        """
        for state in traj:
            flag = False
            if self.robot.type == "Circular2D":
                flag = self.check_circular(state, self.boundary)
            elif self.robot.type == "Rectangular2D":
                flag = self.check_rectangular(state, self.boundary)

            if not flag:
                return False

        return True
