#this file implements my own test for the 2d bspline problems
#and my own attempt to recreate them to understand how it works

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from bsplinegenerator.bsplines import BsplineEvaluation

from path_generation.path_generator import PathGenerator
from path_generation.safe_flight_corridor import SFC, SFC_Data, plot_sfcs, get2DRotationAndTranslationFromPoints
from path_generation.obstacle import Obstacle, plot_2D_obstacles
from path_generation.waypoint_data import plot2D_waypoints
from path_generation.waypoint_data import Waypoint, WaypointData
from path_generation.path_plotter import set_axes_equal
import time

#creates the waypoints case 1
waypoint_a = Waypoint(location=np.array([[0.0],[0.0]]), velocity=np.array([[0.0],[1.0]]))
waypoint_b = Waypoint(location=np.array([[40.0],[10.0]]), velocity=np.array([[20.0],[0.0]]))
waypoint_data_1 = WaypointData(start_waypoint=waypoint_a, end_waypoint=waypoint_b)

obstacle_1 = None

#sets the order. TODO I need to figure out whether this is degree or order.
order = 3
#sets our dimension to 2d
dimensions = 2

#sets the number of intervals in free space
#TODO. Now, I do not know what this means. This could mean the number of intervals of the spline
#which is used to create the control points, etc. 
num_intervals_free_space = 5

#sets the max curvature, TODO, I think this is the inverse of the instantaneous radius
max_curvature = 0.5

#sets the start time of the thing
spline_start_time = 0.0

#sets the scale factor
scale_factor = 1.0

#sets the number of sample points
numSamplePoints = 10000

#creates the path generator
path_gen = PathGenerator(dimension=dimensions,
                         num_intervals_free_space=num_intervals_free_space)

obstacles = None

objective_function_type="minimal_distance_path"

#creates the start time
start_time = time.time()
#creates the control points based on our constraints
control_points, status = path_gen.generate_path(waypoint_data=waypoint_data_1, 
                                                max_curvature=max_curvature,
                                                max_incline=None,
                                                sfc_data=None,
                                                obstacles=obstacles,
                                                objective_function_type=objective_function_type)

#gets the end time after running this optimization funciton
end_time = time.time()
#gets the number of control points
num_ctrl_pnts = np.shape(control_points)[1]
#gets the evaluation time
evaluation_time = end_time - start_time
print("Evaluation Time: ", evaluation_time)

#Now that we have the control points and parameters, we can construct
#the actual BSpline
B_Spline = BsplineEvaluation(control_points=control_points,
                             order=order,
                             start_time=spline_start_time,
                             scale_factor=scale_factor,
                             clamped=False)#sets that this is a natural, not a clamped B_Spline

#gets the spline data and the time data from the function
spline_data, time_data = B_Spline.get_spline_data(numSamplePoints)
#gets the path length
path_length = round(B_Spline.get_arc_length(numSamplePoints),2)
waypoints = waypoint_data_1.get_waypoint_locations()

#creates the subplot
fig, ax = plt.subplots(1,1)
ax.plot(spline_data[0,:], spline_data[1,:])
ax.set_xlabel("x (m) \n \n evaluation time: " + str(np.round(evaluation_time,2)) + 
                   "\n path length: " + str(path_length) +
                   "\n num ctrl pts: " + str(num_ctrl_pnts))
ax.set_aspect('equal')
plot2D_waypoints(waypoint_data_1, ax,arrow_scale = 2)
if obstacles is not None:
    plot_2D_obstacles(obstacles, ax)



ax.set_title("Zero Obstacles")
plt.show()