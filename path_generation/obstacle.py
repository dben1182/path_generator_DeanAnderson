import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class Obstacle:
    center: np.ndarray
    radius: np.double

def plot_2D_obstacle(obstacle: Obstacle, ax):
    circle = plt.Circle((obstacle.center.item(0), obstacle.center.item(1)), 
                        obstacle.radius, color='r')
    ax.add_patch(circle)

def plot_3D_obstacle(obstacle: Obstacle, ax):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = obstacle.radius * np.cos(u)*np.sin(v) + obstacle.center.item(0)
    y = obstacle.radius * np.sin(u)*np.sin(v) + obstacle.center.item(1)
    z = obstacle.radius * np.cos(v) + obstacle.center.item(2)
    ax.plot_surface(x, y, z, color="r")

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])