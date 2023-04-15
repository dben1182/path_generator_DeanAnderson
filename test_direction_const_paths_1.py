import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from bsplinegenerator.bsplines import BsplineEvaluation
from path_generation.path_generator import PathGenerator
from path_generation.safe_flight_corridor import SFC_3D, plot_3D_sfcs, get3DRotationAndTranslationFromPoints
import time
from path_generation.obstacle import set_axes_equal

point_1 = np.array([[3],[4],[0]])
point_2 = np.array([[7],[8],[13]])
dimension = np.shape(point_1)[0]
max_curvature = 1
max_incline = None

# create first path
waypoint_directions_1 = np.array([[1,0],[1,-1],[0,0]])
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
scale_factor = 1.1959952205291566
bspline_1 = BsplineEvaluation(control_points_1, order, spline_start_time, scale_factor, False)
end_time = bspline_1.get_end_time()
end_velocity = bspline_1.get_derivative_at_time_t(end_time,1)
end_acceleration = bspline_1.get_derivative_at_time_t(end_time,2)



print("optimization time 1: " , end_time_1 - start_time_1)




spline_data_1, time_data_1 = bspline_1.get_spline_data(number_data_points)
curvature_data_1, time_data_1 = bspline_1.get_spline_curvature_data(number_data_points)

plt.figure()
ax = plt.axes(projection='3d')
ax.plot(spline_data_1[0,:], spline_data_1[1,:],spline_data_1[2,:], color = "r")
ax.scatter(point_sequence_1[0,:],point_sequence_1[1,:],point_sequence_1[2,:], color = "r")
set_axes_equal(ax)
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
plt.show()

# plt.figure()
# plt.title("Curvature")
# plt.plot(time_data_1, curvature_data_1)
# plt.show()

bspline_1.plot_derivative_magnitude(number_data_points,1)

