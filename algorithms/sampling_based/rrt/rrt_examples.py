import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from algorithms.sampling_based.rrt.rrt_naive import RRTNaive
from algorithms.sampling_based.utils.steer_functions import *
from algorithms.sampling_based.utils.distance_functions import *
from algorithms.sampling_based.utils.collision_checkers import CollisionCheckerGrid
from algorithms.sampling_based.utils.state_samplers import StateSamplerUniform
from algorithms.sampling_based.utils.nearest_neighbor import NNBruteForce

from environment.worlds.occupancy_grid import OccupancyGrid
from environment.robots.turtlebot import Turtlebot
from environment.robots.car import BicycleCar


def rrt_naive_example(steer_type="diff_drive"):
    # create an occupancy grid
    res = 0.01
    grid_map = OccupancyGrid("../../../resources/occ_maps/test_map.png", res)
    map_width = grid_map.width * res
    map_height = grid_map.height * res

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(0, map_width)
    ax.set_ylim(0, map_height)

    # visualize map
    grid_map.visualize(ax)

    # create a robot
    robot = Turtlebot(size=0.15)

    # create a sampler without goal bias
    sampler = StateSamplerUniform((0, 0, -np.pi), (map_width, map_height, np.pi), goal_bias=0.05)

    # create a collision checker
    checker = CollisionCheckerGrid(grid_map, robot)

    # create a steer function
    if steer_type == "diff_drive":
        sfunc = SteerFunctionPOSQ()
    else:
        sfunc = SteerFunctionNaive()

    # create a nearest neighbor searcher
    nn = NNBruteForce(euclidean_dist)

    # create the RRT
    if steer_type == "diff_drive":
        rrt = RRTNaive(sampler, checker, nn, sfunc, inc=0.25)
    else:
        rrt = RRTNaive(sampler, checker, nn, sfunc, inc=0.2)

    # set goal and end position
    rrt.set_init(np.array([0.5, 0.5, 0.0]))
    rrt.set_goal(np.array([6.5, 5.5, 0.0]), np.array([0.1, 0.1]))
    sampler.set_goal(np.array([6.5, 5.5, 0.0]))

    # plot the goal and end position
    ax.add_patch(
        patches.Circle(
            (0.5, 0.5), 0.1,
            facecolor="blue", alpha=0.5
        )
    )
    ax.add_patch(
        patches.Circle(
            (6.5, 5.5), 0.1,
            facecolor="red", alpha=0.5
        )
    )

    # plan
    rrt.solve(max_iter=2000, visualize=True, ax=ax)

    # show plots
    plt.show()


if __name__ == "__main__":
    # rrt_naive_example("geometry")
    rrt_naive_example("diff_drive")
