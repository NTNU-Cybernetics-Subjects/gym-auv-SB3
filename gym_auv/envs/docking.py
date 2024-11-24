
import numpy as np
import random
import math

from objects.dock import RectangularDock, TetrisDock
from objects.rewarder import DockingRewarder, DockingRewarderAdvanced

import gym_auv.utils.geomutils as geom
from gym_auv.objects.vessel import Vessel
from gym_auv.objects.path import RandomCurveThroughOrigin, Path
from gym_auv.objects.obstacles import CircularObstacle, VesselObstacle
from gym_auv.environment import BaseEnvironment
from gym_auv.utils import helpers

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
    """Simple environment with only the dock at a fixed position."""
    
    def _generate(self) -> None:
        # print("Calling _generate on DockingTestScenario0")
        # self.path = Path([[0, 0, 50, 100, 150], [0, 50, 100, 150, 200]])
        # self.path = Path([[0, 1], [0, 0]])
        self.path = None
        self._rewarder_class = DockingRewarderAdvanced

        # init_state = self.path(0)
        # init_angle = self.path.get_direction(0)
        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # This is y,x ??
        dock_pos = (75, -50)
        dock_angle = -np.pi/10

        # Initialize dock at random position within sensor_range
        self.dock = TetrisDock(dock_pos, dock_angle, 30., 25.)

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

    # def observe(self): # # pyright: ignore
    #     # return super().observe()()
    #     navigation_states = self.vessel.navigate(self.path)
    #     if bool(self.config["sensing"]):
    #         perception_states = self.vessel.perceive(self.obstacles, dock=self.dock)
    #     else:
    #         perception_states = []
    #
    #     obs = {"perception": perception_states, "navigation": navigation_states}
    #     return obs
        
    # def _save_latest_step(self): # FIXME: Overrides BaseEnvironment function (THIS IS A QUICKFIX)
    #     pass


class DockingRandomDockScenario0(BaseEnvironment):
    """Simple environment with only the dock."""
    
    def _generate(self) -> None:
        # print("Calling _generate on DockingTestScenario0")
        # self.path = Path([[0, 0, 50, 100, 150], [0, 50, 100, 150, 200]])
        # self.path = Path([[0, 1], [0, 0]])
        self.path = None
        self._rewarder_class = DockingRewarder

        # init_state = self.path(0)
        # init_angle = self.path.get_direction(0)
        init_state = (0,0)
        init_angle = np.pi/2

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # Initialize dock at random position within sensor_range
        sensor_range = self.config["sensor_range"]
        min_distance = 25
        print(f"Initialize dock between {min_distance} and {sensor_range}")
        dock_position, dock_angle = helpers.get_random_dock_position(min_distance, sensor_range)
        self.dock = TetrisDock(dock_position, dock_angle, 25., 20.)





