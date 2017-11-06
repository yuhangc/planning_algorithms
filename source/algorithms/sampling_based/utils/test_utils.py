import matplotlib.pyplot as plt
import numpy as np

from source.environment.worlds.occupancy_grid import OccupancyGrid
from source.environment.robots.turtlebot import Turtlebot
from source.environment.robots.car import BicycleCar
from source.algorithms.sampling_based.utils.state_samplers import StateSamplerUniform
from source.algorithms.sampling_based.utils.collision_checkers import CollisionCheckerGrid
from source.algorithms.sampling_based.utils.steer_functions import SteerFunctionPOSQ


def test_collision_checker_car():
    # create an occupancy grid
    grid_map = OccupancyGrid("../../../resources/occ_maps/test_map.png", 0.01)

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(0, 7.5)
    ax.set_ylim(0, 6)

    # visualize map
    grid_map.visualize(ax)

    robot = BicycleCar(size=(0.2, 0.4))
    # robot = Turtlebot(size=0.15)

    # create a collision checker
    checker = CollisionCheckerGrid(grid_map, robot)

    # check the test state
    test_state = np.array([4.28, 1.83, 1.9])
    valid = checker.check(test_state, ax)

    # visualize robot
    robot.set_pose(test_state)
    robot.visualize(ax)

    if valid:
        ax.scatter(test_state[0], test_state[1], color="white", s=50)
    else:
        ax.scatter(test_state[0], test_state[1], color="red", s=50)

    plt.show()


def test_sampler_collision_checker(robot_type):
    # create an occupancy grid
    grid_map = OccupancyGrid("../../../resources/occ_maps/test_map.png", 0.01)

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(0, 7.5)
    ax.set_ylim(0, 6)

    # visualize map
    grid_map.visualize(ax)

    # create a robot
    if robot_type == "Car":
        robot = BicycleCar(size=(0.2, 0.4))
    elif robot_type == "Turtlebot":
        robot = Turtlebot(size=0.15)

    # create a sampler without goal bias
    sampler = StateSamplerUniform((0, 0, -np.pi), (7.5, 6.0, np.pi))
    # create a collision checker
    checker = CollisionCheckerGrid(grid_map, robot)

    # randomly sample 10 states and check collision
    for k in range(20):
        sample_state = sampler.sample()
        valid = checker.check(sample_state)

        # draw the robot
        if robot_type == "Car":
            robot_draw = BicycleCar(size=(0.2, 0.4))
        elif robot_type == "Turtlebot":
            robot_draw = Turtlebot(size=0.15)
        robot_draw.set_pose(sample_state)
        robot_draw.visualize(ax)

        if valid:
            ax.scatter(sample_state[0], sample_state[1], color="white", s=50)
        else:
            ax.scatter(sample_state[0], sample_state[1], color="red", s=50)

    plt.show()


def test_steer_function():
    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)

    # create a steering function
    sfunc = SteerFunctionPOSQ()

    # set a bunch of test goals
    xe_list = np.array([[3.0, 0.0, 0.0],
                   [2.5, 2.5, 0.0],
                   [0.0, 3.0, 0.5 * np.pi],
                   [2.5, -2.5, -0.5 * np.pi],
                   [-2.5, 2.5, -np.pi]])

    # test for reaching goal within gamma
    for xe in xe_list:
        traj = sfunc.steer(np.array([0.0, 0.0, 0.0]), xe)

        # plot trajectory
        ax.plot(traj[:, 0], traj[:, 1], '-o', markersize=3, fillstyle='none')

    plt.show()


def test_collision_check_traj():
    # create an occupancy grid
    grid_map = OccupancyGrid("../../../resources/occ_maps/test_map.png", 0.01)

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(0, 7.5)
    ax.set_ylim(0, 6)

    # visualize map
    grid_map.visualize(ax)

    # robot = BicycleCar(size=(0.2, 0.4))
    robot = Turtlebot(size=0.15)

    # create a collision checker
    checker = CollisionCheckerGrid(grid_map, robot)

    # create a steering function
    sfunc = SteerFunctionPOSQ()

    # starting and ending position
    xs = np.array([2.0, 2.0, 0.0])
    xe = np.array([4.0, 1.5, 0.0])

    robot.set_pose(xs)
    robot.visualize(ax)

    # obtain trajectory
    traj = sfunc.steer(xs, xe)
    ax.plot(traj[:, 0], traj[:, 1], '-o', markersize=3, fillstyle='none')

    # check for collision
    valid = checker.check_traj(traj)
    print "Trajectory has collision: ", not valid

    plt.show()


if __name__ == "__main__":
    # test_sampler_collision_checker("Turtlebot")
    # test_sampler_collision_checker("Car")
    # test_collision_checker_car()
    # test_steer_function()
    test_collision_check_traj()
