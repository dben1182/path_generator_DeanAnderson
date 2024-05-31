import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from bsplinegenerator.bsplines import BsplineEvaluation
# from path_generation.path_generator import PathGenerator
from path_generation.path_generator import PathGenerator
from path_generation.safe_flight_corridor import SFC_Data, get3DRotationAndTranslationFromPoints
from path_generation.path_plotter import set_axes_equal
from path_generation.waypoint_data import Waypoint, WaypointData, plot3D_waypoints
from path_generation.obstacle import Obstacle, plot_3D_obstacles, plot_3D_obstacle
import time

def create_random_waypoint(acceleration:np.ndarray = None):
    position = np.random.rand(3,1)*np.array([[500],[500],[200]]) + np.array([[0],[0],[10]])
    velocity = np.random.rand(3,1)*np.array([[1],[1],[0]])
    direction = velocity/np.linalg.norm(velocity)
    waypoint = Waypoint(location=position)
    waypoint.velocity = direction
    if acceleration is not None:
        waypoint.acceleration = acceleration
    return waypoint

def check_if_point_is_far_enough_from_obstacle(obstacle_: Obstacle, r_path: float, point:np.ndarray):
    distance = np.linalg.norm(point.flatten()[0:2] - obstacle_.center.flatten()[0:2])
    r_obst = obstacle_.radius
    h = np.abs(point.item(2) - obstacle_.center.item(2))
    if h > r_obst:
        return True
    else:
        r_cut = r_obst*np.sin(np.arccos(h/r_obst))
        min_distance = r_path / np.tan(np.arcsin((r_path)/(r_cut + r_path)))
        if distance < min_distance:
            return False
        else:
            return True

def create_random_waypoint_obstacle_free(obstacle_: Obstacle, r_path: float, acceleration:np.ndarray = None):
    waypoint = create_random_waypoint(acceleration)
    while not check_if_point_is_far_enough_from_obstacle(obstacle_, r_path, waypoint.location):
        waypoint = create_random_waypoint(acceleration)
    return waypoint

def create_curvature_continuous_waypoint(bspline: BsplineEvaluation, end_spline_time: float):
    position = bspline.get_spline_at_time_t(end_spline_time)
    velocity = bspline.get_derivative_at_time_t(end_spline_time,1)
    vel_norm = np.linalg.norm(velocity)
    acceleration = bspline.get_derivative_at_time_t(end_spline_time,2)
    direction = velocity * (1/vel_norm)
    accel_direction = acceleration * (1/vel_norm)**2
    waypoint = Waypoint(location=position)
    waypoint.velocity = direction
    waypoint.acceleration = accel_direction
    return waypoint

def set_velocity_scale(waypoint_a: Waypoint, waypoint_b: Waypoint, custom_scale:float=None):
    distance = np.linalg.norm(waypoint_a.location - waypoint_b.location)
    if custom_scale is None:
        scale = distance/1.5
    else:
        scale = custom_scale
    vel_norm_a = np.linalg.norm(waypoint_a.velocity)
    vel_norm_b = np.linalg.norm(waypoint_b.velocity)
    waypoint_a.velocity = waypoint_a.velocity/vel_norm_a * scale
    waypoint_a.acceleration = waypoint_a.acceleration/(vel_norm_a**2) *scale**2
    waypoint_b.velocity = waypoint_b.velocity/vel_norm_b *scale

##### Initial Settings
dimension = 3
max_curvature = 0.0114362
r_min = 1/max_curvature
max_incline = 0.2
order = 3
scale_factor = 1
path_objective_type = "minimal_velocity_path"
obstacle_type="sphere"
obstacle_radius = 50
obstacle_center = np.array([[250],[250],[150]])
obstacle = Obstacle(center=obstacle_center, radius=obstacle_radius)
obstacles = [obstacle] 
# obstacles = None
number_data_points = 10000
num_intervals_free_space = 8
path_gen = PathGenerator(dimension, num_intervals_free_space=num_intervals_free_space)

### 1st path
acceleration = np.array([[0],[0],[0]])
waypoint_0 = create_random_waypoint_obstacle_free(obstacle, r_min, acceleration)
waypoint_1 = create_random_waypoint_obstacle_free(obstacle, r_min)
set_velocity_scale(waypoint_0, waypoint_1)
waypoint_data_1 = WaypointData(start_waypoint=waypoint_0,end_waypoint=waypoint_1)
start_time_1 = time.time()
control_points_1, success_1 = path_gen.generate_path(waypoint_data=waypoint_data_1, max_curvature=max_curvature,
    max_incline=max_incline, sfc_data=None, obstacles=obstacles, objective_function_type=path_objective_type, 
    obstacle_type=obstacle_type)
end_time_1 = time.time()
np.save("control_points_1", control_points_1)
path_time_1 = end_time_1 - start_time_1
bspline_1 = BsplineEvaluation(control_points_1, order, 0, scale_factor, False)
time_spline_1 = bspline_1.get_end_time()


#### 2nd path 
waypoint_1 = create_curvature_continuous_waypoint(bspline_1, time_spline_1)
waypoint_2 = create_random_waypoint_obstacle_free(obstacle, r_min)
set_velocity_scale(waypoint_1, waypoint_2)
waypoint_data_2 = WaypointData(start_waypoint=waypoint_1, end_waypoint=waypoint_2)
start_time_2 = time.time()
control_points_2, success_2 = path_gen.generate_path(waypoint_data=waypoint_data_2, max_curvature=max_curvature,
    max_incline=max_incline, sfc_data=None, obstacles=obstacles, objective_function_type=path_objective_type, 
    obstacle_type=obstacle_type)
end_time_2 = time.time()
np.save("control_points_2", control_points_2)
path_time_2 = end_time_2 - start_time_2
bspline_2 = BsplineEvaluation(control_points_2, order, 0, scale_factor, False)
time_spline_2 = bspline_2.get_end_time()

##### 3rd path
waypoint_2 = create_curvature_continuous_waypoint(bspline_2, time_spline_2)
waypoint_3 = create_random_waypoint_obstacle_free(obstacle, r_min)
set_velocity_scale(waypoint_2, waypoint_3)
waypoint_data_3 = WaypointData(start_waypoint=waypoint_2, end_waypoint=waypoint_3)
start_time_3 = time.time()
control_points_3, success_3 = path_gen.generate_path(waypoint_data=waypoint_data_3, max_curvature=max_curvature,
    max_incline=max_incline, sfc_data=None, obstacles=obstacles, objective_function_type=path_objective_type, 
    obstacle_type=obstacle_type)
end_time_3 = time.time()
np.save("control_points_3", control_points_3)
path_time_3 = end_time_3 - start_time_3
bspline_3 = BsplineEvaluation(control_points_3, order, 0, scale_factor, False)
time_spline_3 = bspline_3.get_end_time()


##### 4th path
waypoint_3 = create_curvature_continuous_waypoint(bspline_3, time_spline_3)
waypoint_4 = create_random_waypoint_obstacle_free(obstacle, r_min)
set_velocity_scale(waypoint_3, waypoint_4)
waypoint_data_4 = WaypointData(start_waypoint=waypoint_3, end_waypoint=waypoint_4)
start_time_4 = time.time()
control_points_4, success_4 = path_gen.generate_path(waypoint_data=waypoint_data_4, max_curvature=max_curvature,
    max_incline=max_incline, sfc_data=None, obstacles=obstacles, objective_function_type=path_objective_type, 
    obstacle_type=obstacle_type)
end_time_4 = time.time()
np.save("control_points_4", control_points_4)
path_time_4 = end_time_4 - start_time_4
bspline_4 = BsplineEvaluation(control_points_4, order, 0, scale_factor, False)
time_spline_4 = bspline_4.get_end_time()


##### 5th path
waypoint_4 = create_curvature_continuous_waypoint(bspline_4, time_spline_4)
waypoint_5 = create_random_waypoint_obstacle_free(obstacle, r_min)
set_velocity_scale(waypoint_4, waypoint_5)
waypoint_data_5 = WaypointData(start_waypoint=waypoint_4, end_waypoint=waypoint_5)
start_time_5 = time.time()
control_points_5, success_5 = path_gen.generate_path(waypoint_data=waypoint_data_5, max_curvature=max_curvature,
    max_incline=max_incline, sfc_data=None, obstacles=obstacles, objective_function_type=path_objective_type, 
    obstacle_type=obstacle_type)
end_time_5 = time.time()
np.save("control_points_5", control_points_5)
path_time_5 = end_time_5 - start_time_5
bspline_5 = BsplineEvaluation(control_points_5, order, 0, scale_factor, False)
time_spline_5 = bspline_5.get_end_time()
spline_data_5, time_data_5 = bspline_5.get_spline_data(number_data_points)


spline_data_1, time_data_1 = bspline_1.get_spline_data(number_data_points)
spline_data_2, time_data_2 = bspline_2.get_spline_data(number_data_points)
spline_data_3, time_data_3 = bspline_3.get_spline_data(number_data_points)
spline_data_4, time_data_4 = bspline_4.get_spline_data(number_data_points)
spline_data_5, time_data_5 = bspline_5.get_spline_data(number_data_points)

curvature_data_1, time_data_1 = bspline_1.get_spline_curvature_data(number_data_points)
curvature_data_2, time_data_2 = bspline_2.get_spline_curvature_data(number_data_points)
curvature_data_3, time_data_3 = bspline_3.get_spline_curvature_data(number_data_points)
curvature_data_4, time_data_4 = bspline_4.get_spline_curvature_data(number_data_points)
curvature_data_5, time_data_5 = bspline_5.get_spline_curvature_data(number_data_points)

incline_data_1, time_data_1 = bspline_1.get_spline_slope_data(number_data_points)
incline_data_2, time_data_2 = bspline_2.get_spline_slope_data(number_data_points)
incline_data_3, time_data_3 = bspline_3.get_spline_slope_data(number_data_points)
incline_data_4, time_data_4 = bspline_4.get_spline_slope_data(number_data_points)
incline_data_5, time_data_5 = bspline_5.get_spline_slope_data(number_data_points)

waypoint_0.print_waypoint(0)
waypoint_1.print_waypoint(1)
waypoint_2.print_waypoint(2)
waypoint_3.print_waypoint(3)
waypoint_4.print_waypoint(4)
waypoint_5.print_waypoint(5)

print("computation time 1: " , path_time_1)
if (max_curvature < np.max(curvature_data_1)):
    print(" max curvature 1: ", np.max(curvature_data_1))
if (max_incline < np.max(np.abs(incline_data_1))):
    print(" max_slope 1: " , np.max(np.abs(incline_data_1)))

print("computation time 2: " , path_time_2)
if (max_curvature < np.max(curvature_data_2)):
    print(" max curvature 2: ", np.max(curvature_data_2))
if (max_incline < np.max(np.abs(incline_data_2))):
    print(" max_slope 2: " , np.max(np.abs(incline_data_2)))

print("computation time 3: " , path_time_3)
if (max_curvature < np.max(curvature_data_3)):
    print(" max curvature 3: ", np.max(curvature_data_3))
if (max_incline < np.max(np.abs(incline_data_3))):
    print(" max_slope 3: " , np.max(np.abs(incline_data_3)))

print("computation time 4: " , path_time_4)
if (max_curvature < np.max(curvature_data_4)):
    print(" max curvature 4: ", np.max(curvature_data_4))
if (max_incline < np.max(np.abs(incline_data_4))):
    print(" max_slope 4: " , np.max(np.abs(incline_data_4)))

print("computation time 5: " , path_time_5)
if (max_curvature < np.max(curvature_data_4)):
    print(" max curvature 5: ", np.max(curvature_data_5))
if (max_incline < np.max(np.abs(incline_data_5))):
    print(" max_slope 5: " , np.max(np.abs(incline_data_5)))



plt.figure()
ax = plt.axes(projection='3d')
ax.plot(spline_data_1[0,:], spline_data_1[1,:],spline_data_1[2,:], color = "b")
ax.plot(spline_data_2[0,:], spline_data_2[1,:],spline_data_2[2,:], color = "r")
ax.plot(spline_data_3[0,:], spline_data_3[1,:],spline_data_3[2,:], color = "g")
ax.plot(spline_data_4[0,:], spline_data_4[1,:],spline_data_4[2,:], color = "tab:olive")
ax.plot(spline_data_5[0,:], spline_data_5[1,:],spline_data_5[2,:], color = "tab:orange")
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
plot_3D_obstacle(obstacle, ax)
plot3D_waypoints(waypoint_data_1, ax)
# plot3D_waypoints(waypoint_data_2, ax)
plot3D_waypoints(waypoint_data_3, ax)
# plot3D_waypoints(waypoint_data_4, ax)
plot3D_waypoints(waypoint_data_5, ax)
set_axes_equal(ax,dimension)
plt.show()

plt.figure()
plt.title("Curvature")
plt.plot(time_data_1, curvature_data_1,color = "b")
plt.plot(time_data_2+time_data_1[-1], curvature_data_2, color = "r")
plt.plot(time_data_3+time_data_2[-1]+time_data_1[-1], curvature_data_3, color = "g")
plt.plot(time_data_4+time_data_2[-1]+time_data_1[-1]+time_data_3[-1], curvature_data_4, color = "tab:olive")
plt.plot(time_data_5+time_data_2[-1]+time_data_1[-1]+time_data_3[-1]+time_data_4[-1], curvature_data_5, color = "tab:orange")
plt.plot(np.array([0,time_data_5[-1]+time_data_2[-1]+time_data_1[-1]+time_data_3[-1]+time_data_4[-1]]), np.array([1,1])*max_curvature, color = 'k')
plt.show()

plt.figure()
plt.title("Slope")
plt.plot(time_data_1, np.abs(incline_data_1),color = "b")
plt.plot(time_data_2+time_data_1[-1], np.abs(incline_data_2), color = "r")
plt.plot(time_data_3+time_data_2[-1]+time_data_1[-1], np.abs(incline_data_3), color = "g")
plt.plot(time_data_4+time_data_2[-1]+time_data_1[-1]+time_data_3[-1], np.abs(incline_data_4), color = "tab:olive")
plt.plot(time_data_5+time_data_2[-1]+time_data_1[-1]+time_data_3[-1]+time_data_4[-1], np.abs(incline_data_5), color = "tab:orange")
plt.plot(np.array([0,time_data_5[-1]+time_data_2[-1]+time_data_1[-1]+time_data_3[-1]+time_data_4[-1]]), np.array([1,1])*max_incline, color = 'k')
plt.show()
