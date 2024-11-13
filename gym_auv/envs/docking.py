
import numpy as np
import random
import math

from objects.dock import RectangularDock, TetrisDock

import gym_auv.utils.geomutils as geom
from gym_auv.objects.vessel import Vessel
from gym_auv.objects.path import RandomCurveThroughOrigin, Path
from gym_auv.objects.obstacles import CircularObstacle, VesselObstacle
from gym_auv.environment import BaseEnvironment

import os
dir_path = os.path.dirname(os.path.realpath(__file__))

TERRAIN_DATA_PATH = 'resources/terrain.npy'

deg2rad = math.pi/180

# class DockingBaseEnvironment(BaseEnvironment):
#     def __init__(self, *args, **kwargs):
#         """Add a dock object before intializing BaseEnvironment"""
#
#         # TODO: Should set the dock object
#         self.dock = None # Set dock
#         print("DockingBaseEnvironment called.")
#
#         super().__init__(*args, **kwargs)


class DockingTestScenario0(BaseEnvironment):
    """Simple environment with only the dock."""
    
    def _generate(self) -> None:
        print("Calling _generate on DockingTestScenario0")
        self.path = Path([[0, 0, 50, 100, 150], [0, 50, 100, 150, 200]])

        init_state = self.path(0)
        init_angle = self.path.get_direction(0)

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # dock_position = self.path.end
        dock_position = (0, 50)
        self.dock = TetrisDock(dock_position, 16., 16.)

        # prog = self.path.get_closest_arclength(self.vessel.position)
        # self.path_prog_hist = np.array([prog])
        # self.max_path_prog = prog

        # obst_arclength = 50
        # for o in range(9):
        #     obst_radius = 20
        #     obst_arclength += obst_radius*2 + 170
        #     obst_position = self.path(obst_arclength)
        #
        #     obst_displacement = np.array([obst_radius*(-1)**(o+1), obst_radius])
        #     self.obstacles.append(CircularObstacle(obst_position + obst_displacement, obst_radius))






