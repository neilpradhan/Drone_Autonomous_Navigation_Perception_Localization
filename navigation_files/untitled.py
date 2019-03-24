import matplotlib.pyplot as plt
import math
import matplotlib.pyplot as plt
import math
import numpy as np

from occupation_grid import *

from multiprocessing import Queue

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


#distance between 2 points
def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d    



def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)



nstart = Node(2,3, 0.0, -1)
ngoal = Node(15,18, 0.0, -1)
ox = []
oy = []



openset, closedset = dict(), dict()
openset[calc_index(nstart,79,0,0)] = nstart


c_id = min(openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
current = openset[c_id]        