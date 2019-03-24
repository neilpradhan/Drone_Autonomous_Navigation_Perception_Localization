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


# we slash original grid to new obmap and now in this obmap grid the numbers will be 0 to total
# calc index will give the box number in this grid(slashed one)
def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion    


motion = get_motion_model()
nstart = Node(5,4, 0.0, -1)
ngoal = Node(12,18, 0.0, -1)
ox = []
oy = []



openset, closedset = dict(), dict()
openset[calc_index(nstart,79,0,0)] = nstart# saves a node index calculated is by that function

#c_id gives minimum index
c_id = min(openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
current = openset[c_id]        

# print(c_id)
# print(openset)
# print(current)
# #print(current)

# print(type(c_id))
# print(type(openset))
# print(type(current))

# print("new")
# print(openset[321])




# for i, _ in enumerate(motion):
# 	print(i,_)


sx=10.0
sy=10.0
gx =50.0
gy = 50.0
m= new_map
print(type(m))

rows,columns = np.where(m == 1)
#print(result)
# print(len(rows))
# print(len(columns))
# print(np.sum(m==-1))
ox = list(rows)
oy = list(columns)
#print(ox)

#print(m[79][79])

	
boolArr = (m == 1)
print(np.sum(boolArr==1))