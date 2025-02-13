#This file is a python wrapper to be able to get a Bspline object from a generated path,
#Which is itself generated from the path_generator class. Let's see what the easiest way to do this is.

from path_generation.path_generator import PathGenerator
from bsplinegenerator.bsplines import BsplineEvaluation

from path_generation.safe_flight_corridor import SFC, SFC_Data, plot_sfcs, get2DRotationAndTranslationFromPoints
from path_generation.obstacle import Obstacle, plot_2D_obstacles
from path_generation.waypoint_data import plot2D_waypoints
from path_generation.waypoint_data import Waypoint, WaypointData
from path_generation.path_plotter import set_axes_equal
import time

import numpy as np
import matplotlib.pyplot as plt

from typing import Optional

#imports the typing-optional thingamajig.


#creates the path spline generator class
class pathSplineGenerator:

    #creates the init function
    #Arguments:
    #1. dimension is the dimension we are working in (usually 2d or 3d)
    #2. num_intervals_free_space:
    #3. degree: the degree of the polynomial we will be using (usually 3rd degree)
    #4. clamped: bool to state whether or not the spline will be a clamped spline, or alternatively a Natural spline
    #5. numSamplePoints: int to set the number of sample points for 
    def __init__(self,
                 dimension: int = 2,
                 num_intervals_free_space: int = 5,
                 degree: int = 3,
                 clamped: bool = False,
                 numSamplePoints: int = 10000):
        
        #saves the numbers
        self.dimension = dimension
        self.num_intervals_free_space = num_intervals_free_space
        self.degree = degree

        #sets the start time
        self.start_time = 0.0
        #sets the scale factor
        self.scale_factor = 1.0
        #saves whether it is clamped
        self.clamped = clamped
        #saves the number of sample points
        self.numSamplePoints = numSamplePoints
        

        #saves the waypoint data
        self.waypoint_data : Optional[WaypointData] = None

        #instantiates the path generator
        self.path_generator = PathGenerator(dimension=dimension,
                                            num_intervals_free_space=num_intervals_free_space)
        
        #instantiates the BSpline evaluation
        self.bspline_evaluator : Optional[BsplineEvaluation] = None

        #saves the evaluation time 
        self.evaluation_time : Optional[float] = None

        #saves the path length
        self.path_length : Optional[float] = None

        #saves the obstacles
        self.obstacles : Optional[list] = None

        #saves the number of control points
        self.num_ctrl_pnts : Optional[int] = None




    #creates the function to generate a path

    def generate_path(self, 
                      waypoint_data: WaypointData, 
                      max_curvature: np.float64 = None,
                      max_incline: np.float64 = None, 
                      sfc_data: SFC_Data = None, 
                      obstacles: list = None,
                      objective_function_type: str = "minimal_velocity_path", 
                      obstacle_type = "circular"):


        #saves the obstacles data
        self.obstacles = obstacles

        #saves the waypoint data
        self.waypoint_data = waypoint_data

        #saves the start time
        start_time = time.time()
        #calls the corresponding path generator class to get the control points
        controlPoints, status = self.path_generator.generate_path(waypoint_data=waypoint_data,
                                                                  max_curvature=max_curvature,
                                                                  max_incline=max_incline,
                                                                  sfc_data=sfc_data,
                                                                  obstacles=obstacles,
                                                                  objective_function_type=objective_function_type,
                                                                  obstacle_type=obstacle_type)
        end_time = time.time()
        self.evaluation_time = end_time - start_time

        self.num_ctrl_pnts = np.shape(controlPoints)[1]


        #gets the BSpline from those control points
        self.bspline_evaluator = BsplineEvaluation(control_points=controlPoints,
                                                   order=self.degree,
                                                   start_time=self.start_time,
                                                   scale_factor=self.scale_factor,
                                                   clamped=self.clamped)
        

        #gets the spline and time data
        spline_data, time_data = self.bspline_evaluator.get_spline_data(num_data_points_per_interval=self.numSamplePoints)

        #gets the total path length
        self.path_length = round(self.bspline_evaluator.get_arc_length(self.numSamplePoints),2)
        #obtains the waypoints
        waypoints = self.waypoint_data.get_waypoint_locations()

        #returns the time_data, spline_data, etc
        return time_data, spline_data

    #'''
    #creates function to plot the data
    def plotSpline(self,
                   time_data: np.ndarray,
                   spline_data: np.ndarray):
        
        #creates the subplot
        fig, ax = plt.subplots(1,1)
        ax.plot(spline_data[0,:], spline_data[1,:])
        ax.set_xlabel("x (m) \n \n evaluation time: " + str(np.round(self.evaluation_time,2)) + 
                           "\n path length: " + str(self.path_length) +
                           "\n num ctrl pts: " + str(self.num_ctrl_pnts))
        ax.set_aspect('equal')
        plot2D_waypoints(self.waypoint_data, ax,arrow_scale = 2)
        if self.obstacles is not None:
            plot_2D_obstacles(self.obstacles, ax)



        ax.set_title("Zero Obstacles")
        plt.show()
    
    #'''
        


