from path_generation.pathSplineGenerator import pathSplineGenerator
import matplotlib.pyplot as plt
from path_generation.waypoint_data import Waypoint, WaypointData
import numpy as np

from path_generation.obstacle import Obstacle, plot_2D_obstacles
from path_generation.waypoint_data import plot2D_waypoints

#instantiates the path spline generator
pathGenerator = pathSplineGenerator(dimension=2,
                                    num_intervals_free_space=5,
                                    degree=3,
                                    clamped=False,
                                    numSamplePoints=10000)

#creates the waypoints
waypoint_a = Waypoint(location=np.array([[0.0],[0.0]]), velocity=np.array([[0.0],[1.0]]))
waypoint_b = Waypoint(location=np.array([[40.0],[10.0]]), velocity=np.array([[20.0],[0.0]]))
waypoint_data_1 = WaypointData(start_waypoint=waypoint_a, end_waypoint=waypoint_b)



time_data, spline_data = pathGenerator.generate_path(waypoint_data=waypoint_data_1)

#calls the plotter function
pathGenerator.plotSpline(spline_data=spline_data, time_data=time_data)

potatoe = 0