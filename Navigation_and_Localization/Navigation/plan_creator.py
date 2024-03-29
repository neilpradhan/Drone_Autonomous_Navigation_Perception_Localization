
#!/usr/bin/env python
import json
import matplotlib.pyplot as plt
import numpy as np
import math
#from occupation_grid import *
#from og_case2 import *
#from multiprocessing import Queues
from grid import *


#import matplotlib.pyplot as plt
#import math


import matplotlib.pyplot as plt
import math


show_animation = True



moving_factor = 0.25
grid_width = width/division# 24
grid_height = height/division# 16



class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)






def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node((sx / reso), (sy / reso), 0.0, -1)
    ngoal = Node((gx / reso), (gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    #obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)
    #print("length of obstacle map",len(obmap))
    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map_for_this(ox, oy, reso, rr)
    #print(obmap)
    motion = get_motion_model()
    #print(calc_index(nstart, xw, minx, miny))
    openset, closedset = dict(), dict()
    openset[calc_index_for_this(nstart, minx, miny)] = nstart
    #print(openset)
    #print(nstart.cost)
    while 1:
    	#print("size obstacle map",size(obmap))#24
    	#print(obmap)
        print(openset)
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]
        print(c_id)

        # show graph
        if show_animation:  # pragma: no cover
            plt.plot(current.x* reso , current.y*reso, "xc")
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

            n_id = calc_index_for_this(node, minx, miny)
            #print(n_id)
            if n_id in closedset:
                continue
            
            if not verify_node(node, obmap, minx, miny, maxx, maxy):
            #if not True:
                #
                continue
                #print("hello")
            if n_id not in openset:
                #print(1000000)
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry

def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x <= minx:
        return False
    elif node.y <= miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False
    
    


    x_grid = int((node.x - minx)/0.25)
    y_grid  = int(16.0-(node.y-miny)/0.25)
    if obmap[y_grid][x_grid]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):
    
    # minx = min(min(ox))
    # miny = min(min(oy))
    # maxx = max(max(ox))
    # maxy = max(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))



    # minx = round(minx)
    # miny = round(miny)
    # maxx = round(maxx)
    # maxy = round(maxy)
    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)
    #xwidth= (xwidth)
    #ywidth = (ywidth)
    # obstacle map generation
    obmap = [[False for i in range(int(ywidth))] for i in range(int(xwidth))]
  
    for ix in range(int(xwidth)):
        x = ix + minx
        for iy in range(int(ywidth)):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = float(((iox - x)**2 + (ioy - y)**2)**0.5)
                if d <= vr/reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_obstacle_map_for_this(ox, oy, reso, vr):
    
    # minx = min(min(ox))
    # miny = min(min(oy))
    # maxx = max(max(ox))
    # maxy = max(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    minx = x_min
    miny = y_min
    maxx = x_max
    maxy = y_max



    # minx = round(minx)
    # miny = round(miny)
    # maxx = round(maxx)
    # maxy = round(maxy)
    xwidth = grid_width #parts
    ywidth = grid_height
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)
    #xwidth= (xwidth)
    #ywidth = (ywidth)
    # obstacle map generation
    obmap = [[False for i in range(int(grid_width)+1)] for i in range(int(grid_height)+1)]
    for ix in range(int(grid_width)+1):
        x = ix*0.25 + minx 
        for iy in range(int(grid_height)+1):
            y = (16-iy)*0.25 + miny
            for iox,ioy in zip(ox,oy):
                d = (((iox - x)**2 + (ioy - y)**2)**(0.5))
                if d == 0.0:
            	    obmap[iy][ix]=True
            	    break
            #  print(x, y)
            # for iox, ioy in zip(ox, oy):
            #     #d = float(((iox - x)**2 + (ioy - y)**2)**0.5)
            #     if d ==0.0:
            #         obmap[ix][iy] = True
            #         break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth



def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)

def calc_index_for_this(node,xmin, ymin):
    return (node.y - ymin)/0.25 * (25.0) + (node.x - xmin)/0.25    


def get_motion_model():
    # dx, dy, cost
    # motion = [[1, 0, 1],
    #           [0, 1, 1],
    #           [-1, 0, 1],
    #           [0, -1, 1],
    #           [-1, -1, math.sqrt(2)],
    #           [-1, 1, math.sqrt(2)],
    #           [1, -1, math.sqrt(2)],
    #           [1, 1, math.sqrt(2)]]
    motion = [[0.25, 0.0, 0.25],
              [0, 0.25, 0.25],
              [-0.25, 0, 0.25],
              [0, -0.25, 0.25],
              [-0.25, -0.25,0.25* math.sqrt(2)],
              [-0.25, 0.25, 0.25*math.sqrt(2)],
              [0.25, -0.25, 0.25*math.sqrt(2)],
              [0.25, 0.25, 0.25*math.sqrt(2)]]  

    # motion = [[0.005, 0.0, 0.005],
    #           [0, 0.005, 0.005],
    #           [-0.005, 0, 0.005],
    #           [0, -0.005, 0.005],
    #           [-0.005, -0.005,0.005* math.sqrt(2)],
    #           [-0.005, 0.005, 0.005*math.sqrt(2)],
    #           [0.005, -0.005, 0.005*math.sqrt(2)],
    #           [0.005, 0.005, 0.005*math.sqrt(2)]]  
    return motion


# def main():
#     print(__file__ + " start!!")

#     # start and goal position
#     sx = -1.5  # [m]
#     sy = 1.0  # [m]
#     gx = -3.00  # [m]
#     gy = 1.0 # [m]
#     grid_size = 1.0  # [m]
#     robot_size = 0.0 # [m]

#     # ox, oy = [], []

#     # for i in range(60):
#     #     ox.append(i)
#     #     oy.append(0.0)
#     # for i in range(60):
#     #     ox.append(60.0)
#     #     oy.append(i)
#     # for i in range(61):
#     #     ox.append(i)
#     #     oy.append(60.0)
#     # for i in range(61):
#     #     ox.append(0.0)
#     #     oy.append(i)
#     # for i in range(40):
#     #     ox.append(20.0)
#     #     oy.append(i)
#     # for i in range(40):
#     #     ox.append(40.0)
#     #     oy.append(60.0 - i)
#     # obmap, minx, miny, maxx, maxy, xwidth, ywidth = calc_obstacle_map_for_this(ox, oy,1.0, 0.0)
#     # print((obmap))
#     # for i in range(17):
#     # 	for j in range(25):
#     # 		if obmap[i][j]==True:
#     # 			obmap[i][j]=1
#     # 		elif obmap[i][j]==False:
#     # 			obmap[i][j]=0	
#     # print(obmap)
#     # #plt.plot(obmap)
#     # plt.imshow(obmap, cmap = "gray")
#     # plt.show()
#     # #plt.grid(obmap)

#     if show_animation:  # pragma: no cover
#         plt.plot(ox, oy, ".k")
#         plt.plot(sx, sy, "xr")
#         plt.plot(gx, gy, "xb")
#         plt.grid(True)
#         plt.axis("equal")

#     rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
#     print("rx",rx)
#     print("ry",ry)
#     if show_animation:  # pragma: no cover
#         plt.plot(rx, ry, "-r")
#         plt.show()


# if __name__ == '__main__':
#     main()
