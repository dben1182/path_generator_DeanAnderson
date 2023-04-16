import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class Waypoint:
    location: np.ndarray
    velocity: np.ndarray = None
    acceleration: np.ndarray = None

    def checkIfDerivativesActive(self):
        return (self.checkIfVelocityActive() or self.checkIfAccelerationActive())
        
    def checkIfVelocityActive(self):
        return (self.velocity is not None)
    
    def checkIfAccelerationActive(self):
        return (self.acceleration is not None)


@dataclass
class WaypointData:
    start_waypoint: Waypoint(location=np.array([]))
    end_waypoint: Waypoint(location=np.array([]))
    # intermediate_locations: np.ndarray = None

    def get_waypoint_locations(self):
        point_sequence = np.concatenate((self.start_waypoint.location,self.end_waypoint.location),1)
        return point_sequence

    # def set_location_data(self, point_sequence):
    #     self.start_waypoint.location = point_sequence[:,0][:,None]
    #     self.end_waypoint.location = point_sequence[:,-1][:,None]
    #     if np.shape(point_sequence)[1] > 2:
    #         self.intermediate_locations = point_sequence[:,1:-1][:,None]
