#!/usr/bin/env python
import json
import matplotlib.pyplot as plt
import numpy as np
import math
from occupation_grid import *

from multiprocessing import Queue


m=new_map# adjacency matrix

#declare the start and end 

## sr and sc found by quadrant_to_grid func from occupation_grid by inputing arg=[x,y] start point in quad
## assume we have sr and sc as element of [0,80) and [0,80)


# first decide enpoint and start point
m[50,50]=3# end point

sr = 0 # start point x
sc = 0 # start point y

rq = Queue()
cq = Queue()

move_count = 0
#global nodes_left_in_layer 
#global nodes_in_next_layer 
#nodes_left_in_layer =1
#nodes_in_next_layer =0


reached_end = False
visited = np.full((80,80),False)
#print(visited)

dr = [-1,1,0,0]
dc = [0,0,1,-1]
rq.put(sr)
cq.put(sc)
visited[sr][sc]=True






def explore_neighbours(r,c):
    global nodes_left_in_layer
    nodes_left_in_layer =1
    global nodes_in_next_layer
    nodes_in_next_layer = 0
    for i in range(0,4):
    	rr = r + dr[i]
    	cc = c + dc[i]
    	# skip locations at corners or out of bounds
    	if (rr<0 or cc<0):
    		continue
    	if (rr>=80 or cc>=80):
    		continue
        #skip blocked cells
        if visited[rr][cc]:
        	continue
        if m[rr][cc] == 1:
        	continue
        rq.put(rr)
        cq.put(cc)
        visited[rr][cc]=True
        nodes_in_next_layer+=1


while not rq.empty():
    r = rq.get()
    c = cq.get()
    if m[r][c]==3:
        reached_end = True
        break
    explore_neighbours(r,c)
    nodes_left_in_layer-=1
    if nodes_left_in_layer==0:
        nodes_left_in_layer == nodes_in_next_layer
        nodes_in_next_layer=0
        move_count+=1
if reached_end:
    print(move_count)
print(-1)    
   
print(visited)


           	