import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class Waypoint:
    location: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

# @dataclass
# class WaypointData:
#     waypoints: list(Waypoint)
