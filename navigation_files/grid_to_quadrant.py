import json
import matplotlib.pyplot as plt
import numpy as np
import math


#arg2 will have [x,y]

def grid_to_quadrant(arg2):
    grid_x=float(arg2[0])
    grid_y=float(arg2[1])
    #x_e = stop[0]
    #y_e = stop[1]


    # 4 cases for 4 quadrants
    if ((0<=grid_x<=40) and (0<=grid_y<=40))  :
        quad_y = 10*(1- grid_x/40)
        quad_x = -10*(1-grid_y/40)

    elif((0<=grid_x<=40) and (40<=grid_y<=80)) :
        quad_y = 10*(1- grid_x/40)
        quad_x = 10*(grid_y-40)/40

    elif((40<=grid_x<=80) and (0<=grid_y<=40))  :
        quad_y = -10*(grid_x-40)/40
        quad_x = -10*(1-grid_y/40)   
       
    elif((40<=grid_x<=80) and (40<=grid_y<=80))  :
        quad_y = -10*(grid_x-40)/40
        quad_x = -10*(grid_y-40)/40
    
    quad_x = float(quad_x)
    quad_y = float(quad_y)
    return [quad_x,quad_y] # quad values in float