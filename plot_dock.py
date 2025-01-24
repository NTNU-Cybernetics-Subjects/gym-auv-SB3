import matplotlib.pyplot as plt
import shapely.geometry
import shapely.affinity
from gym_auv.objects.dock import TetrisDock
import numpy as np

dock = TetrisDock([0, 0], 0.0, 40.0, 50.0)

# print(f"dock object: {dock.boundary}, dock good zone: {dock.get_good_zone()}")

obstacles = list(dock.boundary.exterior.coords)
good_zone = list(dock.get_good_zone().exterior.coords)


def format_points(points, scale=10):
    def scale_point(point):
        x = point[1] / scale
        y = point[0] / scale
        return (x, y)

    test = [str(scale_point(p)) for p in points]

    p = " -- ".join(test)
    print(f"\draw[fill=gray!40, draw=black]{p} -- cycle;")


def get_x_n_y(points):
    x = [p[1] for p in points]
    y = [p[0] for p in points]
    return x, y

obs = shapely.affinity.translate(dock.boundary, xoff=4, yoff=-1.25)
obstacles = list(obs.exterior.coords)
format_points(points=obstacles)
# format_points(points=good_zone)
# print(f"good zone center {dock.get_good_zone_center()}")
#
# points = [(0,-0.7), (0.3,0.5), (0.3,-0.5), (-0.3,-0.5), (-0.3,0.5), (0,-0.7)]
# b = shapely.geometry.Polygon(points)
# new_b = shapely.affinity.translate(b, xoff=1, yoff=1)
# format_points(list(new_b.exterior.coords))

# placed_boundary = shapely.affinity.translate(rotated_boundary, xoff=self._position[0], yoff=self._position[1])

# print(f"\draw[fill=gray!40, draw=black]")
# plt.scatter(*get_x_n_y(obstacles))
# plt.show()

# o_y, o_x = dock.boundary.exterior.xy
# o_y = np.array(o_y) / 10
# o_x = np.array(o_x) / 10
# plt.plot(o_x, o_y)
# plt.scatter(o_x, o_y)
#
# dock_center = (0, -1)
# plt.scatter(dock_center[0], dock_center[1])
#
# boat_coords = [
#     (0, 0),
#     (-0.5, -0.5),
#     (-2, -0.5),
#     (-2, 0.5),
#     (-0.5, 0.5)
# ]
#
# boat = shapely.geometry.Polygon(boat_coords)
# boat = shapely.affinity.rotate(boat, np.pi/4, use_radians=True, origin=(0,0))
# # boat = shapely.affinity.translate(boat, xoff=1.25, yoff=-3)
# boat_center = (0, 0)
#
# b_x, b_y = boat.exterior.xy
# b_x = [x +1.25 for x in b_x]
# b_y = [y -4 for y in b_y]
#
#
#
#
# plt.plot(b_x,b_y)
# plt.scatter(b_x, b_y)
# plt.scatter(boat_center[0], boat_center[1])
#
#
# plt.show()
