import matplotlib.pyplot as plt

from occupancy_grid import OccupancyGrid


def test_map_loading():
    # create an occupancy grid
    grid_map = OccupancyGrid("../../resources/occ_maps/test_map.png", 0.01)

    # create a plot axis
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(0, 7.5)
    ax.set_ylim(0, 6)

    # visualize map
    grid_map.visualize(ax)

    plt.show()


if __name__ == "__main__":
    test_map_loading()
