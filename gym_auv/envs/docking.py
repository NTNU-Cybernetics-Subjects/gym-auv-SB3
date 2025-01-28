
import numpy as np
import random
import math

from gym_auv.objects.dock import RectangularDock, TetrisDock, SimpleDock, SimpleDockWAngle
from gym_auv.objects.rewarder import DockingRewarder, DockingRewarderAdvanced, DockingStraightRewarder, DockingPenelizerRewarder, DockingPenelizerRewarderForSimpleDock, StrandRewarder

import gym_auv.utils.geomutils as geom
from gym_auv.objects.vessel import Vessel
from gym_auv.objects.path import RandomCurveThroughOrigin, Path
from gym_auv.objects.obstacles import CircularObstacle, VesselObstacle, LineObstacle, PolygonObstacle
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
        self._rewarder_class = DockingRewarder

        # init_state = self.path(0)
        # init_angle = self.path.get_direction(0)
        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # This is y,x ??
        # dock_pos = (75, -50)
        dock_pos = (75, 0)
        # dock_angle = -np.pi/10
        dock_angle = 0

        # Initialize dock at random position within sensor_range
        self.dock = TetrisDock(dock_pos, dock_angle, 40., 50.)

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

class DockingTestScenario1(BaseEnvironment):
    """Simple environment with only the dock at a fixed position."""
    
    def _generate(self) -> None:
        # print("Calling _generate on DockingTestScenario0")
        # self.path = Path([[0, 0, 50, 100, 150], [0, 50, 100, 150, 200]])
        # self.path = Path([[0, 1], [0, 0]])
        self.path = None
        self._rewarder_class = DockingRewarder

        # init_state = self.path(0)
        # init_angle = self.path.get_direction(0)
        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # This is y,x ??
        # dock_pos = (75, -50)
        dock_pos = (75, -30)
        # dock_angle = -np.pi/10
        dock_angle = -np.pi/6

        # Initialize dock at random position within sensor_range
        self.dock = TetrisDock(dock_pos, dock_angle, 40., 50.)


class DockingRandomDockScenario0(BaseEnvironment):
    """Simple environment with only the dock."""
    def _generate(self) -> None:
        # print("Calling _generate on DockingTestScenario0")
        # self.path = Path([[0, 0, 50, 100, 150], [0, 50, 100, 150, 200]])
        # self.path = Path([[0, 1], [0, 0]])
        self.path = None
        # self._rewarder_class = DockingRewarder
        self._rewarder_class = DockingPenelizerRewarder

        # init_state = self.path(0)
        # init_angle = self.path.get_direction(0)
        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # Initialize dock at random position within sensor_range
        sensor_range = self.config["sensor_range"]
        min_distance = 25
        # print(f"Initialize dock between {min_distance} and {sensor_range}")
        # dock_position, dock_angle = helpers.get_random_dock_position(min_distance, sensor_range)
        # dock_y_pos = np.random.uniform(8,8)
        dock_y_pos = 4
        dock_pos = (35, dock_y_pos)
        dock_angle = np.random.uniform(0,np.pi/6) * np.sign(dock_y_pos)
        # dock_angle = np.random.uniform(-np.pi/16, np.pi/16)
        # dock_angle = 0

        self.dock = TetrisDock(dock_pos, dock_angle, 35., 25.)

class DockingRandomDockScenario1(BaseEnvironment):
    """Simple environemnt with only the dock spawing at random position within 
    a radius of the sensor_range"""
    
    def _generate(self) -> None:
        # print("Calling _generate on DockingTestScenario0")
        # self.path = Path([[0, 0, 50, 100, 150], [0, 50, 100, 150, 200]])
        # self.path = Path([[0, 1], [0, 0]])
        self.path = None
        self._rewarder_class = DockingPenelizerRewarder

        # init_state = self.path(0)
        # init_angle = self.path.get_direction(0)
        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # Initialize dock at random position within sensor_range
        sensor_range = self.config["sensor_range"]
        min_distance = 25
        print(f"Initialize dock between {min_distance} and {sensor_range}")
        dock_position, dock_angle = helpers.get_random_dock_position(min_distance, sensor_range)
        self.dock = TetrisDock(dock_position, dock_angle, 35., 25.)


class DockingStraightScenario0(BaseEnvironment):
    def _generate(self) -> None:
        self.path = None
        self._rewarder_class = DockingStraightRewarder

        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        dock_position = (35,0)
        dock_angle = 0
        self.dock = TetrisDock(dock_position, dock_angle, 35., 25.)

class DockingStraightVariationScenario0(BaseEnvironment):

    Levels = [
        TetrisDock((35,0), 0, 30.,25.),
        TetrisDock((35,-2), 0, 30.,25.),
        TetrisDock((35,2), 0, 30.,25.),
        TetrisDock((35,-4), 0, 30.,25.),
        TetrisDock((35,4), 0, 30.,25.)
    ]
    def _generate(self) -> None:
        self.path = None
        self._rewarder_class = DockingPenelizerRewarder

        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # dock_position = (35,0)
        # dock_angle = 0
        # dock_position, dock_angle = self._update_dock_position()
        # self.dock = TetrisDock(dock_position, dock_angle, 30., 25.)
        level = self._check_level()
        print(f"Dock level: {level}")
        self.dock = self.Levels[level]


    def _check_level(self):
        reached_goal = self.history["reached_goal"]
        
        # first episode
        if len(reached_goal) <= 0:
            return 0
        else:
            return int(np.sum(reached_goal)/4) % len(self.Levels)


class DockingStraightVariationScenario1(BaseEnvironment):

    Levels = [
        TetrisDock((35,0), 0, 30.,25.),
        TetrisDock((35,-2), -np.pi/8, 30.,25.),
        TetrisDock((35,2), np.pi/8, 30.,25.),
        TetrisDock((35,-4), -np.pi/6, 30.,25.),
        TetrisDock((35,4), np.pi/6, 30.,25.)
    ]
    def _generate(self) -> None:
        self.path = None
        self._rewarder_class = DockingPenelizerRewarder

        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))

        # dock_position = (35,0)
        # dock_angle = 0
        # dock_position, dock_angle = self._update_dock_position()
        # self.dock = TetrisDock(dock_position, dock_angle, 30., 25.)
        level = self._check_level()
        print(f"Dock level: {level}")
        self.dock = self.Levels[level]


    def _check_level(self):
        reached_goal = self.history["reached_goal"]
        
        # first episode
        if len(reached_goal) <= 0:
            return 0
        else:
            return int(np.sum(reached_goal)/4) % len(self.Levels)
        
        
class SimpleDockTestScenario0(BaseEnvironment):
    """Simple environment with only the dock at a fixed position."""
    
    def _generate(self) -> None:
        self.path = None
        self._rewarder_class = DockingPenelizerRewarderForSimpleDock

        init_state = (0,0)
        init_angle = 0

        self.obstacles = []
        
        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))
        
                # circular obstacles
        num_obstacles = 1
        for _ in range(num_obstacles):
            obst_position = helpers.get_random_position_around_boat(50, 55)
            obst_radius = 5   # np.random.uniform(5, 7)
            self.obstacles.append(CircularObstacle(obst_position, obst_radius))

        # This is y,x ??
        # dock_pos = (75, -50)
        dock_pos = (100, 0)

        # Initialize dock, position, width, height
        self.dock = SimpleDock(dock_pos, 4., 4.)


class SimpleDockTestScenario1(BaseEnvironment):
    """Simple environment with the dock at a fixed position, 
        either to left, right, front or back, always 75 m distance."""
    
    def _generate(self) -> None:
        self.path = None
        self._rewarder_class = DockingPenelizerRewarderForSimpleDock

        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))
        self.obstacles = []
        
        # circular obstacles
        num_obstacles = 7
        for _ in range(num_obstacles):
            obst_position = helpers.get_random_position_around_boat(50, 55)
            obst_radius = 5   # np.random.uniform(5, 7)
            self.obstacles.append(CircularObstacle(obst_position, obst_radius))
            
        
        # Make dock spawn either in front, back, left or right of the vessel
        dock_pos = helpers.get_dock_position_front_back_left_or_right(100)

        # # OBSTACLES AT FIXED POSITIONS
        # # circular obstacles always in front, back, left and right of vessel
        # obst_distance = 50
        # obst_radius = 5
        # front = (obst_distance, 0)
        # front_w_offset = (obst_distance, 15)
        # front_w_offset2 = (obst_distance, -15)
        # back = (-obst_distance, 0)
        # back_w_offset = (-obst_distance, 15)
        # back_w_offset2 = (-obst_distance, -15)
        # left = (0, obst_distance)
        # left_w_offset = (15, obst_distance)
        # left_w_offset2 = (-15, obst_distance)
        # right = (0, -obst_distance)
        # right_w_offset = (15, -obst_distance)
        # right_w_offset2 = (-15, -obst_distance)
        
        # # circular obstacles
        # self.obstacles.append(CircularObstacle(front, obst_radius))
        # self.obstacles.append(CircularObstacle(back, obst_radius))
        # self.obstacles.append(CircularObstacle(left, obst_radius))
        # self.obstacles.append(CircularObstacle(right, obst_radius))
        # self.obstacles.append(CircularObstacle(front_w_offset, obst_radius))
        # self.obstacles.append(CircularObstacle(back_w_offset, obst_radius))
        # self.obstacles.append(CircularObstacle(left_w_offset, obst_radius))
        # self.obstacles.append(CircularObstacle(right_w_offset, obst_radius))
        # self.obstacles.append(CircularObstacle(back_w_offset2, obst_radius))
        # self.obstacles.append(CircularObstacle(left_w_offset2, obst_radius))
        # self.obstacles.append(CircularObstacle(right_w_offset2, obst_radius))
        # self.obstacles.append(CircularObstacle(front_w_offset2, obst_radius))
            
        # Initialize dock, position, width, height
        self.dock = SimpleDock(dock_pos, 4., 4.)
        
class NyhavnaScenario0(BaseEnvironment):
    """Environment looking like Nyhavna. Boat spawning in the middle of a square,
        and dock at random position along the square."""
    
    def _generate(self) -> None:
        self.path = None
        self._rewarder_class = DockingPenelizerRewarderForSimpleDock  # DockingPenelizerRewarderForSimpleDock
        self.obstacles = []

        init_state = (0,0)
        init_angle = 0

        self.vessel = Vessel(self.config, np.hstack([init_state, init_angle]))
        
        square_side_size = 101
        
        # Polygon in the shape of a bar bar in front of boat
        poly_pos1 = (square_side_size, -square_side_size)
        poly_pos2 = (square_side_size, square_side_size)
        poly_pos3 = (square_side_size + 100, square_side_size)
        poly_pos4 = (square_side_size + 100, -square_side_size)
        poly_obstacle = PolygonObstacle([poly_pos1, poly_pos2, poly_pos3, poly_pos4])
        self.obstacles.append(poly_obstacle)
        
        # Poly in the shape of a bar in left of boat
        poly_pos1 = (square_side_size, -square_side_size)
        poly_pos2 = (square_side_size, -square_side_size - 100)
        poly_pos3 = (-square_side_size, -square_side_size - 100)
        poly_pos4 = (-square_side_size, -square_side_size)
        poly_obstacle = PolygonObstacle([poly_pos1, poly_pos2, poly_pos3, poly_pos4])
        self.obstacles.append(poly_obstacle)
        
        # Poly in the shape of a bar in right of boat
        poly_pos1 = (square_side_size, square_side_size)
        poly_pos2 = (square_side_size, square_side_size + 100)
        poly_pos3 = (-square_side_size, square_side_size + 100)
        poly_pos4 = (-square_side_size, square_side_size)
        poly_obstacle = PolygonObstacle([poly_pos1, poly_pos2, poly_pos3, poly_pos4])
        self.obstacles.append(poly_obstacle)
        
        # Poly in the shape of a bar in back of boat
        poly_pos1 = (-square_side_size, -square_side_size)
        poly_pos2 = (-square_side_size - 100, -square_side_size)
        poly_pos3 = (-square_side_size - 100, square_side_size)
        poly_pos4 = (-square_side_size, square_side_size)
        poly_obstacle = PolygonObstacle([poly_pos1, poly_pos2, poly_pos3, poly_pos4])
        self.obstacles.append(poly_obstacle)
        
        
        
        pos1 = (square_side_size, -square_side_size)
        pos2 = (square_side_size, square_side_size)
        pos3 = (square_side_size, -square_side_size)
        line_obstacle = LineObstacle([pos1, pos2])
        self.obstacles.append(line_obstacle)

        
        # Place dock randomly along the rectangle
        dock_pos1 = ((square_side_size - 2), np.random.uniform(-(square_side_size - 2), (square_side_size - 2)))
        dock_pos2 = (-(square_side_size - 2), np.random.uniform(-(square_side_size - 2), (square_side_size - 2)))
        dock_pos3 = (np.random.uniform(-(square_side_size - 2), (square_side_size - 2)), (square_side_size - 2))
        dock_pos4 = (np.random.uniform(-(square_side_size - 2), (square_side_size - 2)), -(square_side_size - 2))
        
        dock_positions = [dock_pos1, dock_pos2, dock_pos3, dock_pos4]
        
        dock_pos = dock_positions[np.random.choice(len(dock_positions))]
        
        
        # # Dock at random position around boat in a circle inside square
        # dock_pos = helpers.get_random_position_around_boat(20, 95)
        
        self.dock = SimpleDock(dock_pos, 4., 4.)
        
        # circular obstacles
        num_obstacles = 12
        for _ in range(num_obstacles):
            obst_position = helpers.get_random_position_around_boat(40, 70)
            obst_radius = 5   # np.random.uniform(5, 7)
            self.obstacles.append(CircularObstacle(obst_position, obst_radius))
            

class StrandBaseCase0(BaseEnvironment):
    """Environment looking like Strand. Boat spawning in the middle of a square,
        and dock at random position along the square."""
    
    def _generate(self) -> None:
        self.path = None
        self._rewarder_class = StrandRewarder  # DockingPenelizerRewarderForSimpleDock
        self.obstacles = []
        
        # Random initial x position between 5.0 and 9.5
        x = np.random.uniform(5.0, 9.5)
        y = np.random.uniform(-4, 4)               
        
        init_pos = (x,y)
        
        if y < 0:
            init_angle = np.random.uniform(150, 180)
        elif y > 0:
            init_angle = np.random.uniform(180, 210)
        else:
            init_angle = 0
            
        init_angle = init_angle*deg2rad

        self.vessel = Vessel(self.config, np.hstack([init_pos, init_angle]))
        
        dock_pos = (0, 0)
        dock_heading = 180*deg2rad
        dock_width = 1
        dock_height = 1
        self.dock = SimpleDockWAngle(dock_pos, dock_width, dock_height, dock_heading)
        
            
        
        
        



