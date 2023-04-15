import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from bsplinegenerator.bsplines import BsplineEvaluation
from path_generation.path_generator import PathGenerator
from path_generation.safe_flight_corridor import SFC_3D, plot_3D_sfcs, get3DRotationAndTranslationFromPoints
import time


point_1 = np.array([[3],[4],[0]])
point_2 = np.array([[7],[8],[13]])
point_3 = np.array([[20],[25],[0]])
dimension = np.shape(point_1)[0]
max_curvature = 2
max_incline = None

# create first path
waypoint_directions_1 = np.array([[1,0],[0,0],[0,0]])
waypoint_accelerations_1 = None
point_sequence_1 = np.concatenate((point_1,point_2),axis=1)
dimension = np.shape(point_sequence_1)[0]
order = 3
number_data_points = 10000
path_gen = PathGenerator(dimension)
total_start_time = time.time()
start_time_1 = time.time()
control_points_1 = path_gen.generate_path(point_sequence_1, waypoint_directions_1, waypoint_accelerations_1, 
                                        max_curvature, max_incline=max_incline)
end_time_1 = time.time()
spline_start_time = 0
bspline_1 = BsplineEvaluation(control_points_1, order, spline_start_time, 1, False)
end_time = bspline_1.get_end_time()
end_velocity = bspline_1.get_derivative_at_time_t(end_time,1)
end_acceleration = bspline_1.get_derivative_at_time_t(end_time,2)

### create second path
unscaled_velocity = control_points_1[:,-1] - control_points_1[:,-3]
unscaled_acceleration = control_points_1[:,-3] - 2*control_points_1[:,-2] + control_points_1[:,-1]
waypoint_directions_2 = np.array([[unscaled_velocity.item(0),0],
                                [unscaled_velocity.item(1),0],
                                [unscaled_velocity.item(2),0]])
waypoint_accelerations_2 = np.array([[unscaled_acceleration.item(0),0],
                                   [unscaled_acceleration.item(1),0],
                                   [unscaled_acceleration.item(2),0]])
print("unscaled_acceleration: " , unscaled_acceleration)
# waypoint_accelerations_2 = None
point_sequence_2 = np.concatenate((point_2,point_3),axis=1)
order = 3
number_data_points = 10000
path_gen = PathGenerator(dimension)
start_time_2 = time.time()
control_points_2 = path_gen.generate_path(point_sequence_2, waypoint_directions_2, waypoint_accelerations_2, 
                                        max_curvature, max_incline=max_incline)

end_time_2 = time.time()
total_end_time = time.time()
print("optimization time 1: " , end_time_1 - start_time_1)
print("optimization time 1: " , end_time_2 - start_time_2)
print("total time: " , total_end_time - total_start_time)
spline_start_time = 0
bspline_2 = BsplineEvaluation(control_points_2, order, spline_start_time, 1, False)



spline_data_1, time_data_1 = bspline_1.get_spline_data(number_data_points)
curvature_data_1, time_data_1 = bspline_1.get_spline_curvature_data(number_data_points)
spline_data_2, time_data_2 = bspline_2.get_spline_data(number_data_points)
curvature_data_2, time_data_2 = bspline_2.get_spline_curvature_data(number_data_points)

# accel_2 = bspline_2.get_derivative_at_time_t(0,2)
accel_2 = control_points_2[:,0] - 2*control_points_2[:,1] + control_points_2[:,2]
print("accel_0 for 2: ", accel_2)

plt.figure()
ax = plt.axes(projection='3d')
ax.plot(spline_data_1[0,:], spline_data_1[1,:],spline_data_1[2,:], color = "r")
# ax.scatter(point_sequence_1[0,:],point_sequence_1[1,:],point_sequence_1[2,:], color = "r")
ax.scatter(control_points_1[0,:],control_points_1[1,:],control_points_1[2,:], facecolors='none', edgecolors= "r")
ax.plot(spline_data_2[0,:], spline_data_2[1,:],spline_data_2[2,:], color = "b")
# ax.scatter(point_sequence_2[0,:],point_sequence_2[1,:],point_sequence_2[2,:], color = "b")
ax.scatter(control_points_2[0,:],control_points_2[1,:],control_points_2[2,:], facecolors='none', edgecolors= "b")
plt.show()

plt.figure()
plt.title("Curvature")
plt.plot(time_data_1, curvature_data_1)
plt.plot(time_data_2 + 5, curvature_data_2)
plt.show()



