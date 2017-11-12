import numpy as np
import matplotlib.pyplot as plt
from time import sleep

from turtlebot import Turtlebot
from car import BicycleCar
from humans import HumanSimple


def test_turtlebot_sim():
    robot = Turtlebot(uncertainty=(0.01, 0.05))

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    plt.ion()

    # motion list
    cmd_vel_list = np.array([[0.0, 0.0, 0.0],
                             [0.5, 0.5, 0.0],
                             [2.5, 0.3, 0.5],
                             [5.0, 0.0, -1.0],
                             [7.0, 0.0, 0.0]])

    # simulation at 100hz
    t_curr = 0.0
    dt = 0.01
    vel_idx = 0
    cmd_vel = np.array([0.0, 0.0])
    for k in range(800):
        # update cmd_vel
        if vel_idx < len(cmd_vel_list) and t_curr >= cmd_vel_list[vel_idx, 0]:
            cmd_vel = cmd_vel_list[vel_idx, 1:]
            vel_idx += 1

        # update robot state
        robot.update(cmd_vel, dt)
        sleep(dt * 0.2)

        # plot at lower frequency
        if k % 5 == 0:
            robot.visualize(ax)
            plt.pause(0.001)

        # update simulation time
        t_curr += dt


def test_car_sim():
    robot = BicycleCar(uncertainty=(0.01, 0.05))

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-3, 8)
    ax.set_ylim(-3, 3)
    plt.ion()

    # motion list
    cmd_vel_list = np.array([[0.0, 0.0, 0.0],
                             [0.5, 0.5, 0.0],
                             [2.5, 0.0, 0.2],
                             [5.0, 0.0, -0.2],
                             [7.5, -0.5, 0.0]])

    # simulation at 100hz
    t_curr = 0.0
    dt = 0.01
    vel_idx = 0
    cmd_vel = np.array([0.0, 0.0])
    for k in range(1000):
        # update cmd_vel
        if vel_idx < len(cmd_vel_list) and t_curr >= cmd_vel_list[vel_idx, 0]:
            cmd_vel = cmd_vel_list[vel_idx, 1:]
            vel_idx += 1

        # update robot state
        robot.update(cmd_vel, dt)
        sleep(dt * 0.2)

        # plot at lower frequency
        if k % 5 == 0:
            robot.visualize(ax)
            plt.pause(0.001)

        # update simulation time
        t_curr += dt


def test_human():
    human = HumanSimple()
    human.set_goal(np.array([0, 6]), 0.8, 10.0)

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-1, 8)
    plt.ion()

    # plot the goal
    ax.plot(0.0, 6.0, 'xr')

    # motion list
    forces = np.array([[0.0, 0.0, 0.0],
                       [1.0, 5, 0.0],
                       [2.0, 0.0, 0.0],
                       [3.0, -5, 0.0],
                       [4.5, 0.0, 0.0]])

    # simulation at 100hz
    t_curr = 0.0
    dt = 0.01
    f_idx = 0
    u = np.array([0.0, 0.0])
    for k in range(1000):
        # update cmd_vel
        if f_idx < len(forces) and t_curr >= forces[f_idx, 0]:
            u = forces[f_idx, 1:]
            f_idx += 1

        # update robot state
        human.update(u, dt)
        sleep(dt * 0.2)

        # plot at lower frequency
        if k % 5 == 0:
            human.visualize(ax)
            plt.pause(0.001)

        # update simulation time
        t_curr += dt


if __name__ == "__main__":
    # test_turtlebot_sim()
    # test_car_sim()
    test_human()
