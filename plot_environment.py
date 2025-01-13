import matplotlib.pyplot as plt
import shapely
import shapely.geometry
import shapely.affinity
import numpy as np
from gym_auv.objects.dock import TetrisDock

class Vector:
    def __init__(self, start_point=(0,0), end_point=(0,0)) -> None:
        self.start = np.array(start_point).reshape(2,1)
        self.end = np.array(end_point).reshape(2,1)

    def rotate(self, angle):
        """RAD"""
        rotation_m = np.array([
            [ np.cos(angle), -np.sin(angle) ],
            [ np.sin(angle), np.cos(angle)]
        ])
        vec_origo = self.end - self.start
        vec_rot = np.dot(rotation_m, vec_origo)
        new_end = self.start + vec_rot
        return Vector(self.start.flatten(), new_end.flatten())
    
    def get_end(self):
        return self.end.flatten()

    def get_start(self):
        return self.start.flatten()
    
    def xy(self):
        x = [ self.start[0], self.end[0] ]
        y = [self.start[1], self.end[1]]
        return x,y
    
    def abs(self):
        vec = self.end - self.start
        return np.linalg.norm(vec)
        

def get_obj_coords(points, name):

    for i in range(len(points)):
        print(f"\coordinate ({name}{i}) at {points[i]};")

def define_coord(arraylike, name):
    latex_string = f"\coordinate ({name}) at {tuple( arraylike )};"
    print(latex_string)


pos = (0,0)
angle = np.pi/2 + np.pi/8
width = 40/10
height = 50/10
dock = TetrisDock(pos, angle, width, height)

attack_vec = Vector((0,-1), (0,3))
attack_vec_rot = attack_vec.rotate(np.pi/8)
# print(attack_vec_rot.get_end())

boat_points = [
    (0,0),
    (-0.5, -0.5),
    (-2, -0.5),
    (-2, 0.5),
    (-0.5, 0.5),
]
boat = shapely.geometry.Polygon(boat_points)
boat_rot = shapely.affinity.rotate(boat, 45)
boat_translate_vec = attack_vec_rot.get_end()*-2
boat_trans = shapely.affinity.translate(boat_rot, xoff=boat_translate_vec[0], yoff=boat_translate_vec[1], zoff=0)

boat_center = boat_translate_vec - np.array([1, 0])
# define_coord(boat_center, "BCO")

attack_vec_full = Vector(boat_center, attack_vec_rot.get_end())
attack_dir = attack_vec_full.get_end()/attack_vec_full.abs()
arc_point = boat_center + attack_dir *2
define_coord(arc_point, "A_P")
plt.scatter(*arc_point)

# xb
xb = Vector(boat_center, boat_center + np.array([2,2]))
# define_coord(xb.get_end(), "x_b")
plt.plot(*xb.xy())

# yb 
yb = xb.rotate(-np.pi/2)
# define_coord(yb.get_end(), "y_b")

# NED
xn = Vector(boat_center, boat_center + np.array([3,0]))
define_coord(xn.get_end(), "y_n")
yn = Vector(boat_center, boat_center + np.array([0,3]))
define_coord(yn.get_end(), "x_n")





dock_c = dock.get_good_zone_center()
dock_obst = shapely.affinity.translate(dock.boundary, xoff=-dock_c[0])
dock_zone = shapely.affinity.translate(dock.get_good_zone(), xoff=-dock_c[0])


plt.plot(*boat_trans.exterior.xy)
# plt.scatter(0,0)

plt.plot(*dock_obst.exterior.xy)
# plt.scatter(0, -4)

bp = list(boat_trans.exterior.coords)
# get_obj_coords(bp, "PB")
# get_coords(list(dock_obst.exterior.coords), "PO")
# get_coords(list(dock_zone.exterior.coords ), "PZ")



plt.xlim([-10,10])
plt.ylim([-10,10])
plt.show()
