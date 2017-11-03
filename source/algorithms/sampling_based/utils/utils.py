import numpy as np


def transform_point_to_world(point, state):
    """
    Transform a point from local coordinate to world/global coordinate frame
    :param point: 2d point - (x, y)
    :param state: 2d pose - (x, y, theta)
    :return: transformed point position
    """
    x, y = point
    x0, y0, th = state

    x_new = x * np.cos(th) - y * np.sin(th) + x0
    y_new = x * np.sin(th) + y * np.cos(th) + y0

    return x_new, y_new
