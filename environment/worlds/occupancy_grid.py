import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import cv2


class OccupancyGrid:
    """
    A 2D deterministic occupancy grid map, use the same format as ROS.
    """
    def __init__(self, image_path, resolution, origin_x=0.0, origin_y=0.0, free_th=0.25, occ_th=0.75):
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.free_th = free_th
        self.occ_th = occ_th

        # load the image as gray scale
        im = cv2.imread(image_path, 0)
        cv2.imshow("map", im)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # set map width and height
        self.height, self.width = im.shape

        # if the cells are free
        # 0 - 1 probability of being occupied
        # -1 - unknown
        self.occupancy = -np.ones((self.width, self.height), dtype=float)

        # read from image
        for x in range(0, self.width):
            for y in range(0, self.height):
                self.occupancy[x, y] = (255.0 - im[self.height - y - 1, x]) / 255.0

        # set colors for visualization
        self.color_occ = np.array([0.0, 0.0, 0.0])
        self.color_unsure = np.array([0.6, 0.6, 0.6])
        self.color_unknown = np.array([0.6, 0.6, 1.0])

    def get_grid_pos(self, x, y):
        grid_x = int(round((x - self.origin_x) / self.resolution))
        grid_y = int(round((y - self.origin_y) / self.resolution))

        return grid_x, grid_y

    def is_free(self, x, y, flag_grid_pos=False):
        """
        Check if a given position is free or not.
        :param x: x coordinate
        :param y: y coordinate
        :param flag_grid_pos: whether (x, y) is grid coordinate
        :return: 1 if free, 0 if occupied and -1 for unknown/out of range?
        """
        if not flag_grid_pos:
            x, y = self.get_grid_pos(x, y)

        # check range
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return -1

        # check occupancy
        if self.occupancy[x, y] == -1:
            return -1
        else:
            return self.occupancy[x, y] < self.free_th

    def visualize(self, ax):
        pts_occ = []
        pts_unsure = []
        pts_unknown = []

        # loop through all grid cells
        for gx in range(0, self.width):
            for gy in range(0, self.height):
                x = gx * self.resolution + self.origin_x
                y = gy * self.resolution + self.origin_y

                if self.occupancy[gx, gy] > self.occ_th:
                    pts_occ.append((x, y))
                elif self.occupancy[gx, gy] > self.free_th:
                    pts_unsure.append((x, y))
                elif self.occupancy[gx, gy] < 0:
                    pts_unknown.append((x, y))

        pts_array = np.array(pts_occ)
        if pts_array.size:
            ax.scatter(pts_array[:, 0], pts_array[:, 1],
                       color=np.tile(self.color_occ, (pts_array.shape[0], 1)),
                       s=1, marker='s', label='obstacles')

        pts_array = np.array(pts_unsure)
        if pts_array.size:
            ax.scatter(pts_array[:, 0], pts_array[:, 1],
                       color=np.tile(self.color_unsure, (pts_array.shape[0], 1)),
                       s=1, marker='s', label='unsure')

        pts_array = np.array([pts_unknown])
        if pts_array.size:
            ax.scatter(pts_array[:, 0], pts_array[:, 1],
                       color=np.tile(self.color_unknown, (pts_array.shape[0], 1)),
                       s=1, marker='s', label='unknown')
