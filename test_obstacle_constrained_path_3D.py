import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from bsplinegenerator.bsplines import BsplineEvaluation
from path_generation.path_generator import PathGenerator
from path_generation.safe_flight_corridor import SFC_3D, plot_3D_sfcs, get3DRotationAndTranslationFromPoints
from path_generation.obstacle import Obstacle, plot_3D_obstacles, set_axes_equal
import time

point_1 = np.array([[3],[4],[0]])
point_2 = np.array([[7],[10],[13]])
points = np.concatenate((point_1,point_2),1)
dimension = np.shape(point_1)[0]

waypoint_directions = np.array([[1,0],[0,0],[0,0]])
waypoint_curvatures = np.array([0,0])
point_sequence = np.concatenate((point_1,point_2),axis=1)
dimension = np.shape(point_sequence)[0]
max_curvature = 0.5
max_incline = None
obstacles = [Obstacle(np.array([[4.5],[5.5],[3]]), 1.0),
             Obstacle(np.array([[4],[8],[9.5]]), 1.5)]
# obstacles = [Obstacle(np.array([[4],[6],[3]]), 1.0)]
order = 3
initial_control_points = None


path_gen = PathGenerator(dimension)
start_time = time.time()
control_points = path_gen.generate_path(point_sequence, waypoint_directions, waypoint_curvatures, 
        max_curvature, max_incline=max_incline, sfcs=None, obstacles=obstacles)
end_time = time.time()
print("computation time: " , end_time - start_time)
print("control_points: " , control_points)
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
# acceleration_data, time_data = bspline.get_spline_derivative_data(1000,2)
minvo_cps = bspline.get_minvo_control_points()
waypoints = np.concatenate((point_sequence[:,0][:,None], point_sequence[:,-1][:,None]),1)

print("max incline" , np.max(incline_data))
print("max curvature" , np.max(curvature_data))
# print("sfcs: " , sfcs)
plt.figure()
ax = plt.axes(projection='3d')
if obstacles != None:
    plot_3D_obstacles(obstacles, ax)
ax.plot(spline_data[0,:], spline_data[1,:],spline_data[2,:])
ax.scatter(waypoints[0,:],waypoints[1,:],waypoints[2,:])
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
set_axes_equal(ax)
# plt.scatter(minvo_cps[0,:],minvo_cps[1,:])
plt.show()



# plt.figure()
# plt.plot(time_data, incline_data)
# plt.show()

# plt.figure()
# plt.plot(time_data, curvature_data)
# plt.show()

# print("start curvature: " , curvature_data[0])
# print("end curvature: " , curvature_data[-1])


