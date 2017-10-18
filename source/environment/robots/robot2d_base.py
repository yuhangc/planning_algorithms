import numpy as np
import matplotlib.patches as patches


class Robot2dBase:
    """
    A base class for simple 2d robot.
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vis = None

    def set_pose(self, new_pose):
        self.x, self.y, self.th = new_pose

    def get_pose(self):
        return self.x, self.y, self.th

    def update(self, u, dt):
        """
        Dynamic update, needs to be implemented by derived classes
        :param u: control input
        :param dt: time interval for update
        """
        raise Exception("update() method must be overidden!")

    def visualize(self, ax):
        """
        Visualizing/plotting the robot, needs to be implemented by derived classes
        :param ax: the axis to plot on
        """
        raise Exception("visualize() method must be overriden!")


class Robot2dCircular(Robot2dBase):
    """
    Base class for circular shaped robots.
    """
    def __init__(self, size=0.3):
        self.radius = size
        super(Robot2dCircular, self).__init__()

    def visualize(self, ax):
        if self.vis is None:
            # create new plot
            vis_body = ax.add_patch(
                patches.Circle(
                    (self.x, self.y), self.radius,
                    facecolor="red", alpha=0.5
                )
            )
            # draw the orientation
            x_plot = np.array([self.x, self.x + self.radius * np.cos(self.th)])
            y_plot = np.array([self.y, self.y + self.radius * np.sin(self.th)])
            vis_rot = ax.plot(x_plot, y_plot, lw=2, ls='-')

            self.vis = (vis_body, vis_rot)
        else:
            # update the current drawing
            vis_body, vis_rot = self.vis

            vis_body.center = (self.x, self.y)
            x_plot = np.array([self.x, self.x + self.radius * np.cos(self.th)])
            y_plot = np.array([self.y, self.y + self.radius * np.sin(self.th)])
            vis_rot[0].set_xdata(x_plot)
            vis_rot[0].set_ydata(y_plot)


class Robot2dRectangular(Robot2dBase):
    """
    Base class for rectangular shaped robots.
    """
    def __init__(self, size=(0.3, 0.5)):
        self.w, self.h = size
        super(Robot2dRectangular, self).__init__()

    def visualize(self, ax):
        # calculate position of the lower-left corner
        x0 = self.x - 0.5 * self.h * np.cos(self.th) - 0.5 * self.w * np.sin(self.th)
        y0 = self.y - 0.5 * self.h * np.sin(self.th) + 0.5 * self.w * np.cos(self.th)

        if self.vis is None:
            # create new plot
            vis_body = ax.add_patch(
                patches.Rectangle(
                    (x0, y0), self.w, self.h, self.th,
                    facecolor="red", alpha=0.5
                )
            )
            # draw the orientation
            x_plot = np.array([self.x, self.x + 0.5 * self.h * np.cos(self.th)])
            y_plot = np.array([self.y, self.y + 0.5 * self.h * np.sin(self.th)])
            vis_rot = ax.plot(x_plot, y_plot, lw=2, ls='-')

            self.vis = (vis_body, vis_rot)
        else:
            # update the current drawing
            vis_body, vis_rot = self.vis

            vis_body.set_xy((x0, y0))
            vis_body._angle = np.rad2deg(self.th)

            x_plot = np.array([self.x, self.x + 0.5 * self.h * np.cos(self.th)])
            y_plot = np.array([self.y, self.y + 0.5 * self.h * np.sin(self.th)])
            vis_rot[0].set_xdata(x_plot)
            vis_rot[0].set_ydata(y_plot)
