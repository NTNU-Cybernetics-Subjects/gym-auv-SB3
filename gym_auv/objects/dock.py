import numpy as np
import shapely.geometry
import shapely.affinity
import gym_auv.utils.geomutils as geom
from abc import ABC, abstractmethod
import copy

class BaseDock(ABC):
    def __init__(self, *args, **kwargs) -> None:
        """Initializes obstacle instance by calling private setup method implemented by
         subclasses of BaseDock and calculating obstacle boundary."""
        # FIXME: Should position be list, tuple or shapely.geometry.point?
        self._position = None
        self._setup(*args, **kwargs)
        self._boundary = self._calculate_boundary()

        if not self._boundary.is_valid:
            self._boundary = self._boundary.buffer(0)
        self._init_boundary = copy.deepcopy(self._boundary)

    @property
    def boundary(self) -> shapely.geometry.Polygon:
        """shapely.geometry.Polygon object used for simulating the
        sensors' detection of the obstacle instance."""
        return self._boundary

    @property
    def init_boundary(self) -> shapely.geometry.Polygon:
        """shapely.geometry.Polygon object used for simulating the
        sensors' detection of the obstacle instance."""
        return self._init_boundary

    # FIXME: Do we need update?
    # def update(self, dt:float) -> None:
    #     """Updates the obstacle according to its dynamic behavior, e.g. 
    #     a ship model and recalculates the boundary."""
    #     has_changed = self._update(dt)
    #     if has_changed:
    #         self._boundary = self._calculate_boundary()
    #         if not self._boundary.is_valid:
    #             self._boundary = self._boundary.buffer(0)

    @abstractmethod
    def _calculate_boundary(self) -> shapely.geometry.Polygon:
        """Returns a shapely.geometry.Polygon instance representing the obstacle
        given its current state."""

    @abstractmethod
    def _setup(self, *args, **kwargs) -> None:
        """Initializes the obstacle given the constructor parameters provided to
        the specific BaseObstacle extension."""

    # @abstractmethod
    # def _update(self, _dt:float) -> bool:
    #     """Performs the specific update routine associated with the obstacle.
    #     Returns a boolean flag representing whether something changed or not.
    #
    #     Returns
    #     -------
    #     has_changed : bool
    #     """
    #     return False

    # @property
    # def heading_taken(self) -> list:
    #     """Returns an array holding the heading of the obstacle at previous timesteps."""
    #     return self._prev_heading

# NOTE: do we need to spesify good and bad regions in the dock object?
class RectangularDock(BaseDock):

    def _setup(self, position, width, height) -> None:
        print(f"Setting up RectangularDock with, pos: {position}, width: {width}, height: {height}")
        self.width = width
        self.height = height
        self._position = position
        self._angle = np.pi/2 # rad

        # TODO: define good dock side?

        # Define dock shape
        self.points = [
            (-self.width/2, -self.height/2),
            (-self.width/2, self.height/2),
            (self.width/2, self.height/2),
            (self.width/2, -self.height/2),
            (-self.width/2, -self.height/2)
        ]

    def _calculate_boundary(self) -> shapely.geometry.Polygon:
        origo_boundary = shapely.geometry.Polygon(self.points)
        rotated_boundary = shapely.affinity.rotate(origo_boundary, self._angle, use_radians=True, origin='centroid')
        placed_boundary = shapely.affinity.translate(rotated_boundary, xoff=self._position[0], yoff=self._position[1])
        return placed_boundary


class TetrisDock(BaseDock):
    def _setup(self, position, width, height) -> None:
        print(f"Setting up TetrisDock with, pos: {position}, width: {width}, height: {height}")
        self.width = width
        self.height = height
        self._position = position
        self._angle = np.pi/2 # rad
        # self._angle = 0 # rad

        # Define dock shape
        y_step = self.height/4
        x_step = self.width/2
        self.points = [
            (0, y_step),
            (-x_step, y_step),
            (-x_step, 2*y_step),
            (x_step, 2*y_step),
            (x_step, -2*y_step),
            (-x_step, -2*y_step),
            (-x_step, -y_step),
            (0, -y_step),
            (0, y_step)
        ]

        # Define good zone
        self.good_zone_center_pos = (-x_step/2, 0)
        self.good_zone_points = [
            (-x_step/2, y_step),
            (-x_step/2, -y_step),
            (x_step/2, -y_step),
            (x_step/2, y_step),
            (-x_step/2, y_step)
        ]

    def get_good_zone(self) -> shapely.geometry.Polygon:
        origo_boundary = shapely.geometry.Polygon(self.good_zone_points)

        dissplaced_x = np.cos(self._angle)*self.good_zone_center_pos[0] - np.sin(self._angle)*self.good_zone_center_pos[1]
        dissplaced_y = np.sin(self._angle) * self.good_zone_center_pos[0] + np.sin(self._angle) * self.good_zone_center_pos[1]

        tranx = self._position[0] + dissplaced_x
        trany = self._position[1] + dissplaced_y

        placed_boundary = shapely.affinity.translate(origo_boundary, xoff=tranx, yoff=trany)
        return placed_boundary


    def _calculate_boundary(self) -> shapely.geometry.Polygon:
        origo_boundary = shapely.geometry.Polygon(self.points)
        rotated_boundary = shapely.affinity.rotate(origo_boundary, self._angle, use_radians=True, origin=(0,0))
        placed_boundary = shapely.affinity.translate(rotated_boundary, xoff=self._position[0], yoff=self._position[1])
        return placed_boundary


