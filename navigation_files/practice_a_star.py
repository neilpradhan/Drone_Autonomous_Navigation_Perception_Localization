import matplotlib.pyplot as plt
import math
import numpy as np

from occupation_grid import *

from multiprocessing import Queue




m=new_map# adjacency matrix (-1 means empty, 1 means filed)


show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.value = value

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.value)


#vr is the robot radius 

def calc_obstacle_map(ox, oy, reso, vr):

     minx = round(min(ox))
     miny = round(min(oy))
     maxx = round(max(ox))
     maxy = round(max(oy))
#     #  print("minx:", minx)
#     #  print("miny:", miny)
#     #  print("maxx:", maxx)
#     #  print("maxy:", maxy)

#     xwidth = round(maxx - minx)
#     ywidth = round(maxy - miny)
#     #  print("xwidth:", xwidth)
#     #  print("ywidth:", ywidth)

#     # obstacle map generation
#     obmap = [[False for i in range(int(xwidth))] for i in range(int(ywidth))]
#     for ix in range(int(xwidth)):
#         x = ix + minx  # grid number from 0
#         for iy in range(int(ywidth)):
#             y = iy + miny
#             #  print(x, y)
#             for iox, ioy in zip(ox, oy):
#                 d = math.sqrt((iox - x)**2 + (ioy - y)**2) #distance from obstacle
#                 if d <= vr:    # robot radius 
#                     obmap[ix][iy] = True
#                     break

#     return obmap, minx, miny, maxx, maxy, xwidth, ywidth





m = new_map

ox =[0,1,2,3,4,4]
oy = [0,1,2,3,4,5]

#print(a)
#print(len(a))

#a = np.asarray(a)
#print(np.shape(a))# 5x4

def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)

def calc_final_path(ngoal, closedset):
    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    value = ngoal.value
    while value != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry
    
def a_star_planning(sx, sy, gx, gy, ox, oy,rr):
    """
    gx: goal x position [m]
    gy: goal y position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx), round(sy), 0.0, -1)#-1 means not on an obstacle 
    ngoal = Node(round(gx), round(gy), 0.0, -1)
    ox = [iox for iox in ox]
    oy = [ioy for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:  # pragma: no cover
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry
