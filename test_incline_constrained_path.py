import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from bsplinegenerator.bsplines import BsplineEvaluation
from path_generation.path_generator import PathGenerator
from path_generation.safe_flight_corridor import SFC_3D, plot_3D_sfcs, get3DRotationAndTranslationFromPoints
from path_generation.obstacle import set_axes_equal
import time

point_1 = np.array([[3],[4],[0]])
point_2 = np.array([[7],[10],[13]])
# point_2 = np.array([[7],[12],[5]])
points = np.concatenate((point_1,point_2),1)
dimension = np.shape(point_1)[0]

waypoint_directions = np.array([[3,0],[0.5,1],[1.5,0]])
waypoint_directions = None
#what happens if constrain vel mag
continuity_constraint = np.array([[3,2],
                                  [0.5,-7],
                                  [1.5,-1]])
# continuity_constraint = np.array([[1,2],
#                                   [0.5,-7],
#                                   [0,2]])
# continuity_constraint = None
point_sequence = np.concatenate((point_1,point_2),axis=1)
dimension = np.shape(point_sequence)[0]
max_curvature = 1
# max_curvature = None
max_incline = 1
# max_incline = None
order = 3
initial_control_points = None


path_gen = PathGenerator(dimension)
start_time = time.time()
control_points = path_gen.generate_path(point_sequence, waypoint_directions, continuity_constraint, 
                                        max_curvature, max_incline=max_incline)
end_time = time.time()
print("computation time: " , end_time - start_time)
# print("control_points: " , control_points)
spline_start_time = 0
scale_factor = 1
bspline = BsplineEvaluation(control_points, order, spline_start_time, scale_factor, False)
number_data_points = 10000
spline_data, time_data = bspline.get_spline_data(number_data_points)
spline_velocity_data, time_data = bspline.get_spline_derivative_data(number_data_points,1)
z_vel_mag = spline_velocity_data[2,:]
horiz_vel_mag = np.linalg.norm(spline_velocity_data[0:2,:],2,0)
incline_data = z_vel_mag/horiz_vel_mag
curvature_data, time_data = bspline.get_spline_curvature_data(number_data_points)
velocity_data, time_data = bspline.get_derivative_magnitude_data(number_data_points,1)
# acceleration_data, time_data = bspline.get_spline_derivative_data(1000,2)
minvo_cps = bspline.get_minvo_control_points()
waypoints = np.concatenate((point_sequence[:,0][:,None], point_sequence[:,-1][:,None]),1)

print("max incline" , np.max(incline_data))
print("max curvature" , np.max(curvature_data))
print("start curvature: " , curvature_data[0])
# print("end curvature: " , curvature_data[-1])

# print("sfcs: " , sfcs)
plt.figure()
ax = plt.axes(projection='3d')
ax.plot(spline_data[0,:], spline_data[1,:],spline_data[2,:])
ax.scatter(control_points[0,:], control_points[1,:],control_points[2,:])
# ax.scatter(waypoints[0,:],waypoints[1,:],waypoints[2,:])
# plt.scatter(minvo_cps[0,:],minvo_cps[1,:])
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
set_axes_equal(ax)
plt.show()

# plt.figure()
# plt.plot(time_data, incline_data)
# plt.show()

# plt.figure()
# plt.plot(time_data, curvature_data)
# plt.show()

plt.figure()
plt.plot(time_data, velocity_data)
plt.show()


