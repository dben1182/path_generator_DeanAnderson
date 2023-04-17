"""
This module generates a 3rd order B-spline path between two waypoints,
waypoint directions, curvature constraint, and adjoining 
safe flight corridors.
"""
import os
import numpy as np
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint, Bounds
from path_generation.matrix_evaluation import get_M_matrix
from PathObjectivesAndConstraints.python_wrappers.objective_functions import ObjectiveFunctions
from PathObjectivesAndConstraints.python_wrappers.curvature_constraints import CurvatureConstraints
from PathObjectivesAndConstraints.python_wrappers.obstacle_constraints import ObstacleConstraints
from PathObjectivesAndConstraints.python_wrappers.incline_constraints import InclineConstraints
from PathObjectivesAndConstraints.python_wrappers.waypoint_constraints import WaypointConstraints
from bsplinegenerator.bspline_to_minvo import get_composite_bspline_to_minvo_conversion_matrix
from path_generation.safe_flight_corridor import SFC_Data, SFC
from path_generation.obstacle import Obstacle
from path_generation.waypoint_data import Waypoint, WaypointData
import time



class PathGenerator:
    """
    This class generates a 3rd order B-spline path between two waypoints,
    waypoint directions, curvature constraint, and adjoining 
    safe flight corridors.
    """

### TODO ####
# 1. remove the scale factor constraint
# 2. add checks to make sure constraints are feasible with eachother
# 3. Change obstacle constraints to check only intervals that have an obstacle in the same SFC
# 4. add constraints to reach intermediate waypoints between start and end waypoint.

    def __init__(self, dimension: int, 
                 num_intervals_free_space: int = 5):
        self._dimension = dimension
        self._order = 3
        self._M = get_M_matrix(self._order)
        self._objective_func_obj = ObjectiveFunctions(self._dimension)
        self._curvature_const_obj = CurvatureConstraints(self._dimension)
        self._waypoint_const_obj = WaypointConstraints(self._dimension)
        self._obstacle_cons_obj = ObstacleConstraints(self._dimension)
        self._num_intervals_free_space = num_intervals_free_space
        if dimension == 3:
            self._incline_const_obj = InclineConstraints()
        
    def generate_path(self, waypoint_data: WaypointData, max_curvature: np.float64 = None,
                max_incline: np.float64 = None, sfc_data: SFC_Data = None, obstacles: list = None,
                objective_function_type: str = "minimal_velocity_path"):
        num_intervals = self.__get_num_intervals(sfc_data)
        point_sequence = self.__get_point_sequence(waypoint_data, sfc_data)
        num_cont_pts = self.__get_num_control_points(num_intervals)
        constraints = self.__get_constraints(num_cont_pts, waypoint_data, max_curvature, 
                                             max_incline, sfc_data, obstacles)
        objectiveFunction = self.__get_objective_function(objective_function_type)
        objective_variable_bounds = self.__create_objective_variable_bounds(num_cont_pts)
        optimization_variables = self.__create_initial_objective_variables(num_cont_pts, point_sequence)
        minimize_options = {'disp': False} #, 'maxiter': self.maxiter, 'ftol': tol}
        # perform optimization
        result = minimize(
            objectiveFunction,
            x0=optimization_variables,
            args=(num_cont_pts,),
            method='SLSQP', 
            bounds=objective_variable_bounds,
            constraints=constraints, 
            options = minimize_options)
        optimized_control_points = np.reshape(result.x[0:-3] ,(self._dimension,num_cont_pts))
        return optimized_control_points
    
    def __get_objective_function(self, objective_function_type):
        if objective_function_type == "minimal_distance_path":
            return self.__minimize_velocity_control_points_objective_function
        elif objective_function_type == "minimal_velocity_path":
            return self.__minimize_acceleration_control_points_objective_function
        elif objective_function_type == "minimal_acceleration_path":
            return self.__minimize_jerk_control_points_objective_function
        else:
            raise Exception("Error, Invalid objective function type")

    def __get_num_intervals(self, sfc_data: SFC_Data):
        num_intervals = self._num_intervals_free_space
        if sfc_data is not None:
            num_intervals = sfc_data.get_num_intervals()
        return num_intervals
        
    def __get_point_sequence(self, waypoint_data:WaypointData, sfc_data:SFC_Data = None):
        if sfc_data is None:
            point_sequence = np.concatenate((waypoint_data.start_waypoint.location,
                                             waypoint_data.end_waypoint.location),1)
            return point_sequence
        else:
            return sfc_data.get_point_sequence()
    
    def __create_initial_objective_variables(self, num_cont_pts, point_sequence):
        control_points = self.__create_initial_control_points(num_cont_pts, point_sequence)
        scale_factor = 1
        start_waypoint_scalar = 1
        final_waypoint_scalar = 1
        variables = np.concatenate((control_points.flatten(),
            [scale_factor, start_waypoint_scalar, final_waypoint_scalar]))
        return variables
        
    def __get_objective_variables(self, variables, num_cont_pts):
        control_points = np.reshape(variables[0:num_cont_pts*self._dimension], \
                    (self._dimension,num_cont_pts))
        scale_factor = variables[num_cont_pts*self._dimension]
        return control_points, scale_factor
    
    def __get_objective_waypoint_scalars(self, variables):
        return variables[-2], variables[-1]

    def __get_constraints(self, num_cont_pts: int, waypoint_data: WaypointData, 
            max_curvature: np.float64, max_incline: np.float64, sfc_data: SFC_Data, obstacles: list):
        waypoints = np.concatenate((waypoint_data.start_waypoint.location, waypoint_data.end_waypoint.location),1)
        waypoint_constraint = self.__create_waypoint_constraint(waypoints, num_cont_pts)
        constraints = [waypoint_constraint]
        if waypoint_data.start_waypoint.checkIfDerivativesActive():
            start_waypoint_derivatives_constraint = self.__create_start_waypoint_derivative_constraints(waypoint_data.start_waypoint, num_cont_pts)
            constraints.append(start_waypoint_derivatives_constraint)
        if waypoint_data.end_waypoint.checkIfDerivativesActive():
            end_waypoint_derivatives_constraint = self.__create_end_waypoint_derivative_constraints(waypoint_data.end_waypoint, num_cont_pts)
            constraints.append(end_waypoint_derivatives_constraint)
        if max_curvature is not None:
            curvature_constraint = self.__create_curvature_constraint(max_curvature, num_cont_pts)
            constraints.append(curvature_constraint)
        if max_incline is not None:
            incline_constraint = self.__create_incline_constraints(max_incline, num_cont_pts)
            constraints.append(incline_constraint)
        if sfc_data is not None:
            sfc_constraint = self.__create_safe_flight_corridor_constraint(sfc_data, num_cont_pts)
            constraints.append(sfc_constraint)
        if (obstacles != None):
            obstacle_constraint = self.__create_obstacle_constraints(obstacles, num_cont_pts)
            constraints.append(obstacle_constraint)
        return tuple(constraints)

# bounds over just the scale factor
    def __create_objective_variable_bounds(self, num_cont_pts):
        lower_bounds = np.zeros(num_cont_pts*self._dimension + 3) - np.inf
        upper_bounds = np.zeros(num_cont_pts*self._dimension + 3) + np.inf
        lower_bounds[num_cont_pts*self._dimension:] = 0.000001
        return Bounds(lb=lower_bounds, ub = upper_bounds)

    def __minimize_jerk_control_points_objective_function(self, variables, num_cont_pts):
        # for third order splines only
        control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
        return self._objective_func_obj.minimize_acceleration_and_time(control_points, 1)
    
    def __minimize_velocity_control_points_objective_function(self, variables, num_cont_pts):
        # for third order splines only
        control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
        velocity_cps =  control_points[:,0:-1] - control_points[:,1:]
        velocity_control_points_squared_sum = np.sum(velocity_cps**2,0)
        objective = np.sum(velocity_control_points_squared_sum)
        return objective
    
    def __minimize_acceleration_control_points_objective_function(self, variables, num_cont_pts):
        # for third order splines only
        control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
        acceleration_cps =  control_points[:,2:] - 2*control_points[:,1:-1] + control_points[:,0:-2]
        accel_control_points_squared_sum = np.sum(acceleration_cps**2,0)
        objective = np.sum(accel_control_points_squared_sum)
        return objective


    def __create_waypoint_constraint(self, waypoints, num_cont_pts):
        num_waypoints = 2
        m = num_waypoints
        n = num_cont_pts
        k = self._order
        d = self._dimension
        constraint_matrix = np.zeros((m*d,n*d))
        Gamma_0 = np.zeros((self._order+1,1))
        Gamma_0[self._order,0] = 1
        Gamma_f = np.ones((self._order+1,1))
        M_Gamma_0_T = np.dot(self._M,Gamma_0).T
        M_Gamma_f_T = np.dot(self._M,Gamma_f).T
        for i in range(self._dimension):
            constraint_matrix[i*m   ,  i*n        : i*n+k+1] = M_Gamma_0_T
            constraint_matrix[i*m+1 , (i+1)*n-k-1 : (i+1)*n] = M_Gamma_f_T
        constraint_matrix = np.concatenate((constraint_matrix,np.zeros((m*d,3))),1)
        constraint = LinearConstraint(constraint_matrix, lb=waypoints.flatten(), ub=waypoints.flatten())
        return constraint

    def __create_start_waypoint_derivative_constraints(self, start_waypoint: Waypoint, num_cont_pts):
        lower_bound = 0
        upper_bound = 0
        if start_waypoint.checkIfVelocityActive():
            start_velocity_desired = start_waypoint.velocity.flatten()
        if start_waypoint.checkIfAccelerationActive():
            start_acceleration_desired = start_waypoint.acceleration.flatten()
        startVelocityIsActive = start_waypoint.checkIfVelocityActive()
        startAccelerationIsActive = start_waypoint.checkIfAccelerationActive()
        def start_waypoint_derivative_constraint_function(variables):
            control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
            start_waypoint_scalar, end_waypoint_scalar = self.__get_objective_waypoint_scalars(variables)
            constraints = np.array([])
            if startVelocityIsActive:
                start_velocity_direction = start_waypoint_scalar*(control_points[:,2] - control_points[:,0])/2
                constraints = start_velocity_direction - start_velocity_desired
            if startAccelerationIsActive:
                start_acceleration_direction = start_waypoint_scalar*start_waypoint_scalar*(control_points[:,0] - 2*control_points[:,1] + control_points[:,2])
                constraints_2 = start_acceleration_direction - start_acceleration_desired
                constraints = np.concatenate((constraints, constraints_2))
            return constraints.flatten()
        start_waypoint_derivative_constraint = NonlinearConstraint(start_waypoint_derivative_constraint_function, lb= lower_bound, ub=upper_bound)
        return start_waypoint_derivative_constraint
    
    def __create_end_waypoint_derivative_constraints(self, end_waypoint: Waypoint, num_cont_pts):
        lower_bound = 0
        upper_bound = 0
        if end_waypoint.checkIfVelocityActive():
            end_velocity_desired = end_waypoint.velocity.flatten()
        if end_waypoint.checkIfAccelerationActive():
            end_acceleration_desired = end_waypoint.acceleration.flatten()
        endVelocityIsActive = end_waypoint.checkIfVelocityActive()
        endAccelerationIsActive = end_waypoint.checkIfAccelerationActive()
        def end_waypoint_derivative_constraint_function(variables):
            control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
            start_waypoint_scalar, end_waypoint_scalar = self.__get_objective_waypoint_scalars(variables)
            constraints = np.array([])
            if endVelocityIsActive:
                end_velocity_direction = end_waypoint_scalar*(control_points[:,-1] - control_points[:,-3])/2
                constraints = end_velocity_direction - end_velocity_desired
            if endAccelerationIsActive:
                end_acceleration_direction = end_waypoint_scalar*end_waypoint_scalar*(control_points[:,-3] - 2*control_points[:,-2] + control_points[:,-1])
                constraints_2 = end_acceleration_direction - end_acceleration_desired
                constraints = np.concatenate((constraints, constraints_2))
            return constraints.flatten()
        end_waypoint_derivative_constraint = NonlinearConstraint(end_waypoint_derivative_constraint_function, lb= lower_bound, ub=upper_bound)
        return end_waypoint_derivative_constraint

    def __create_curvature_constraint(self, max_curvature, num_cont_pts):
        def curvature_constraint_function(variables):
            control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
            return self._curvature_const_obj.get_spline_curvature_constraint(control_points,max_curvature)
        lower_bound = - np.inf
        upper_bound = 0
        curvature_constraint = NonlinearConstraint(curvature_constraint_function , lb = lower_bound, ub = upper_bound)
        return curvature_constraint
    
    def __create_incline_constraints(self, max_incline, num_cont_pts):
        def incline_constraint_function(variables):
            control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
            constraint = self._incline_const_obj.get_spline_incline_constraint(control_points, 1, max_incline)
            return constraint
        lower_bound = - np.inf
        upper_bound = 0
        incline_constraint = NonlinearConstraint(incline_constraint_function , lb = lower_bound, ub = upper_bound)
        return incline_constraint

    def __create_safe_flight_corridor_constraint(self, sfc_data: SFC_Data, num_cont_pts):
        # create the rotation matrix.
        num_corridors = self.__get_num_corridors(sfc_data)
        num_minvo_cont_pts = (num_cont_pts - self._order)*(self._order+1)
        intervals_per_corridor = sfc_data.get_intervals_per_corridor()
        sfc_list = sfc_data.get_sfc_list()
        M_rot = self.get_composite_sfc_rotation_matrix(intervals_per_corridor, sfc_list, num_minvo_cont_pts)
        # create the bspline to minvo conversion matrix 
        M_minvo = get_composite_bspline_to_minvo_conversion_matrix(\
            num_cont_pts, self._order)
        zero_block = np.zeros((num_minvo_cont_pts,num_cont_pts))
        zero_col = np.zeros((num_minvo_cont_pts, 3))
        if self._dimension == 2:
            M_minvo = np.block([[M_minvo, zero_block, zero_col],
                                        [zero_block, M_minvo, zero_col]])
        if self._dimension == 3:
            M_minvo = np.block([[M_minvo,    zero_block, zero_block, zero_col],
                                [zero_block, M_minvo   , zero_block, zero_col],
                                [zero_block, zero_block, M_minvo   , zero_col]])
        conversion_matrix = M_rot @ M_minvo
        #create bounds
        lower_bounds = np.zeros((self._dimension, num_minvo_cont_pts))
        upper_bounds = np.zeros((self._dimension, num_minvo_cont_pts))
        index = 0
        for corridor_index in range(num_corridors):
            num_intervals = intervals_per_corridor[corridor_index]
            lower_bound, upper_bound = sfc_list[corridor_index].getRotatedBounds()
            num_points = num_intervals*(self._order+1)
            lower_bounds[:,index:index+num_points] = lower_bound
            upper_bounds[:,index:index+num_points] = upper_bound
            index = index+num_points
        safe_corridor_constraints = LinearConstraint(conversion_matrix, lb=lower_bounds.flatten(), ub=upper_bounds.flatten())
        return safe_corridor_constraints
    
    def __create_obstacle_constraints(self, obstacles, num_cont_pts):
        def obstacle_constraint_function(variables):
            control_points, scale_factor = self.__get_objective_variables(variables, num_cont_pts)
            # return self._obstacle_cons_obj.getObstacleConstraintsForIntervals(control_points, obstacle.radius, obstacle.center)
            radii = np.zeros(len(obstacles))
            centers = np.zeros((self._dimension,len(obstacles)))
            for i in range(len(obstacles)):
                radii[i] = obstacles[i].radius
                centers[0,i] = obstacles[i].center[0,0]
                centers[1,i] = obstacles[i].center[1,0]
                if self._dimension == 3:
                    centers[2,i] = obstacles[i].center[2,0]
            return self._obstacle_cons_obj.getObstaclesConstraintsForSpline(control_points, radii, centers)
        lower_bound = 0
        upper_bound = np.inf
        obstacle_constraint = NonlinearConstraint(obstacle_constraint_function , lb = lower_bound, ub = upper_bound)
        return obstacle_constraint
    
    def get_composite_sfc_rotation_matrix(self, intervals_per_corridor, sfcs, num_minvo_cont_pts):
        num_corridors = len(intervals_per_corridor)
        M_len = num_minvo_cont_pts*self._dimension
        M_rot = np.zeros((M_len, M_len))
        num_cont_pts_per_interval = self._order + 1
        interval_count = 0
        dim_step = num_minvo_cont_pts
        for corridor_index in range(num_corridors):
            rotation = sfcs[corridor_index].rotation.T
            num_intervals = intervals_per_corridor[corridor_index]
            for interval_index in range(num_intervals):
                for cont_pt_index in range(num_cont_pts_per_interval):
                    index = interval_count*num_cont_pts_per_interval+cont_pt_index
                    M_rot[index, index] = rotation[0,0]
                    M_rot[index, index + dim_step] = rotation[0,1]
                    M_rot[index + dim_step, index] = rotation[1,0]
                    M_rot[index + dim_step, index + dim_step] = rotation[1,1]
                    if self._dimension == 3:
                        M_rot[2*dim_step + index, index] = rotation[2,0]
                        M_rot[2*dim_step + index, index + dim_step] = rotation[2,1]
                        M_rot[2*dim_step + index, index + 2*dim_step] = rotation[2,2]
                        M_rot[dim_step + index, index + 2*dim_step] = rotation[1,2]
                        M_rot[index, index + 2*dim_step] = rotation[0,2]
                interval_count += 1
        return M_rot

    def __create_initial_control_points(self, total_num_cont_pts, point_sequence):
        num_segments = np.shape(point_sequence)[1] - 1
        if num_segments < 2:
            start_point = point_sequence[:,0]
            end_point = point_sequence[:,1]
            control_points = np.linspace(start_point,end_point,total_num_cont_pts).T
        else:
            control_points = np.empty(shape=(self._dimension,total_num_cont_pts))
            distances = np.linalg.norm(point_sequence[:,1:] - point_sequence[:,0:-1],2,0)
            total_distance = np.sum(distances)
            distance_between_cps = total_distance/(total_num_cont_pts-1)
            point_index = 0
            start_cont_pt_index = 0
            for i in range(total_num_cont_pts):
                interval_start_point = point_sequence[:,point_index]
                interval_end_point = point_sequence[:,point_index+1]
                vector_to_point = interval_end_point - interval_start_point
                unit_vector_to_point = vector_to_point / (np.linalg.norm(vector_to_point))
                num_steps = i-start_cont_pt_index
                control_points[:,i] = unit_vector_to_point * distance_between_cps * i
                if (np.linalg.norm(vector_to_point) <  distance_between_cps*num_steps):
                    point_index += 1
                    start_cont_pt_index = i + 1
                # distance = distances[i] - remainder_distance
                # num_cont_pts = int(np.ceil(distance / distance_between_cps))
                # # if i == len(distances) - 1:
                # #     num_cont_pts -= 2
                # remainder_distance = distance % distance_between_cps
                # segment_control_points = np.linspace(point_sequence[:,i],point_sequence[:,i+1],
                #                                      num_cont_pts).T
                # control_points = np.concatenate((control_points, segment_control_points),1)
        return control_points
        
    def __get_num_control_points(self, num_intervals):
        num_control_points = num_intervals + self._order
        return int(num_control_points)
    
    def __get_num_corridors(self, sfc_data:SFC_Data = None):
        if sfc_data is None:
            return int(0)
        else:
            return sfc_data.get_num_corridors()
        