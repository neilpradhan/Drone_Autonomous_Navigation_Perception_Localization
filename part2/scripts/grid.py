#!/usr/bin/python

"""
Header Files
"""
import matplotlib.pyplot as plt
import math
import numpy as np
import json
from pprint import pprint
"""
Key value are dicts or lists
"""
division = 0.25

def line_y_cordinate_array(arr_x,p1,p2):
	y_arr=[]
	x1 = p1[0]
	y1 = p1[1]
	x2 = p2[0]
	y2 = p2[1]

	for x in arr_x:
		 y = y1 + ((y2-y1)/(x2-x1))*(x-x1)
	y_arr.append(y)
	return y_arr	 

def line_x_cordinate_array(arr_y,p1,p2):
	x_arr=[]
	x1 = p1[0]
	y1 = p1[1]
	x2 = p2[0]
	y2 = p2[1]

	for y in arr_y:
		 x = x1 + ((x2-x1)/(y2-y1))*(y-y1)
	y_arr.append(y)
	return y_arr	

def Equidistantpoints(p1,p2):
	ax=[] #sum of ax1 and ax2
	ay=[] #sum of ay1 and ay2
	x1 = p1[0]
	y1 = p1[1]
	x2 = p2[0]
	y2 = p2[1]

	width  = abs(x1-x2)
	height = abs(y1-y2)

	minx = min(x1,x2)
	miny = min(y1,y2)


	x_parts = width/division # 8 parts
	y_parts = height/division # 3 parts

	ax1= [minx + division*i for i in range(int(x_parts)+1)]
	# these are necessary points in the grid
	ay1=line_y_cordinate_array(ax1,p1,p2)

	ay2 = [miny + division*i for i in range(int(y_parts)+1)]

	ax2 = line_x_cordinate_array(ay2,p1,p2)


	ax = ax1+ax2
	ay = ay1+ ay2

	return ax,ay




def line(p1,p2):
	x_obs = []
	y_obs = []
	x1 = p1[0]
	y1 = p1[1]
	x2 = p2[0]
	y2 = p2[1]

	width  = abs(x1-x2)
	height = abs(y1-y2)

	minx = min(x1,x2)
	miny = min(y1,y2)

	if width==0.0 and height!=0.0:
		y_parts = height/division
		y_obs = [miny + 0.25*i for i in range(int(y_parts)+1)]
		x_obs = [minx for i in range(int(y_parts)+1)]
	elif width !=0.0 and height ==0.0:
		x_parts = width/division
		x_obs = [minx+ 0.25*i for i in range(int(x_parts)+1)]
		y_obs = [miny for y in range(int(x_parts)+1)]

	elif width !=0.0 and height !=0.0:
						
		x_parts = width/division
		y_parts = height/division
		for i in range(int(y_parts)+1):
			for j in range(int(x_parts)+1):
				x_obs.append(minx+division*j)
				y_obs.append(miny+division*i)

	return x_obs,y_obs


def gate_side_lines(p,angle):
	x_obs = []
	y_obs = []
	new_p1 = [0,0]
	new_p2 = [0,0]
	new_p3 = [0,0]
	new_p4 = [0,0]

	if angle == -90.0:
		a = 90.0
	else:
		a = angle


	if a == 135.0:
		new_p1[0] = p[0]-division
		new_p1[1] = p[1]-division
		new_p2[0] = new_p1[0]-division
		new_p2[1] =	new_p1[1]-division

		LINE1_X,LINE1_Y = line(new_p1,new_p2)
		x_obs= x_obs+LINE1_X
		y_obs= y_obs+LINE1_Y

		new_p3[0] = p[0]+division
		new_p3[1] = p[1]+division
		new_p4[0] = new_p3[0]+division
		new_p4[1] =	new_p3[1]+division

		LINE2_X,LINE2_Y = line(new_p3,new_p4)
		x_obs= x_obs+LINE2_X
		y_obs= y_obs+LINE2_Y


	if a == 45.0:
		new_p1[0] = p[0]+division
		new_p1[1] = p[1]-division
		new_p2[0] = new_p1[0]+division
		new_p2[1] =	new_p1[1]-division

		LINE1_X,LINE1_Y = line(new_p1,new_p2)
		x_obs= x_obs+LINE1_X
		y_obs= y_obs+LINE1_Y

		new_p3[0] = p[0]-division
		new_p3[1] = p[1]+division
		new_p4[0] = new_p3[0]-division
		new_p4[1] =	new_p3[1]+division

		LINE2_X,LINE2_Y = line(new_p3,new_p4)
		x_obs= x_obs+LINE2_X
		y_obs= y_obs+LINE2_Y


	if a == 90.0:
		new_p1[0] = p[0]
		new_p1[1] = p[1]-division
		new_p2[0] = new_p1[0]
		new_p2[1] =	new_p1[1]-division

		LINE1_X,LINE1_Y = line(new_p1,new_p2)
		x_obs= x_obs+LINE1_X
		y_obs= y_obs+LINE1_Y

		new_p3[0] = p[0]
		new_p3[1] = p[1]+division
		new_p4[0] = new_p3[0]
		new_p4[1] =	new_p3[1]+division

		LINE2_X,LINE2_Y = line(new_p3,new_p4)
		x_obs= x_obs+LINE2_X
		y_obs= y_obs+LINE2_Y




	if a == 0.0:
		new_p1[0] = p[0]+division
		new_p1[1] = p[1]
		new_p2[0] = new_p1[0]+division
		new_p2[1] =	new_p1[1]

		LINE1_X,LINE1_Y = line(new_p1,new_p2)
		x_obs= x_obs+LINE1_X
		y_obs= y_obs+LINE1_Y

		new_p3[0] = p[0]-division
		new_p3[1] = p[1]
		new_p4[0] = new_p3[0]-division
		new_p4[1] =	new_p3[1]

		LINE2_X,LINE2_Y = line(new_p3,new_p4)
		x_obs= x_obs+LINE2_X
		y_obs= y_obs+LINE2_Y					


	if a == 180.0:
		new_p1[0] = p[0]+division
		new_p1[1] = p[1]
		new_p2[0] = new_p1[0]+division
		new_p2[1] =	new_p1[1]

		LINE1_X,LINE1_Y = line(new_p1,new_p2)
		x_obs= x_obs+LINE1_X
		y_obs= y_obs+LINE1_Y

		new_p3[0] = p[0]-division
		new_p3[1] = p[1]
		new_p4[0] = new_p3[0]-division
		new_p4[1] =	new_p3[1]

		LINE2_X,LINE2_Y = line(new_p3,new_p4)
		x_obs= x_obs+LINE2_X
		y_obs= y_obs+LINE2_Y


	return x_obs,y_obs





with open("/home/neil/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/nav_challenge.world.json") as file:
    data = json.load(file)
    airspace = data["airspace"]
    gates = data["gates"]
    gpoint = []
    gangle = []
    for i in range(len(gates)):
        gpoint.append([gates[i]['position'][0], gates[i]['position'][1]]) # point = [x,y]
        gangle.append(gates[i]['heading']) # angle = [135]
        
    #print(gangle) # gate points[[]]    
    #print(gpoint) # gate angles []
    walls = data["walls"]
    lmin = airspace["min"]
    lmax = airspace["max"]
    wstart = [walls[0]['plane']['start'], walls[1]['plane']['start']]
    wstop = [walls[0]['plane']['stop'], walls[1]['plane']['stop']]

    w_st1 = [wstart[0][0], wstart[0][1]]
    w_sp1 = [wstop[0][0], wstop[0][1]]
    w_st2 = [wstart[1][0], wstart[1][1]]
    w_sp2 = [wstop[1][0], wstop[1][1]]
    #print(gangle) # all heading angles for gates
    #print(lmin) # 3d grid minimum point

#lets make the grid

x_min = lmin[0]# -4
y_min = lmin[1]# -2
z_min = lmin[2]
x_max = lmax[0]# +2
y_max = lmax[1]# +2
z_max = lmax[2]
#print(x_min,y_min,x_max,y_max)
width = x_max - x_min #6.0 
height = y_max - y_min #4.0


grid_width = width/division# 24
grid_height = height/division# 16





# 1. lets make borders first
ox , oy = [], []
# just for addition

for i in range(int(grid_width)+1):
	ox.append(x_min+division*i)
	oy.append(y_min)

# print(len(ox))
# print(ox) # 25 points including start and end but divide line segment into 24 pts
# print(oy)


for i in range(int(grid_width)+1):
	ox.append(x_min+division*i)
	oy.append(y_max)

for i in range(int(grid_height)+1):
	ox.append(x_min)
	oy.append(y_min+division*i)

for i in range(int(grid_height)+1):
	ox.append(x_max)
	oy.append(y_min+division*i)



#wall_1 = list(Equidistantpoints(w_st1,w_sp1)) # points will be [(x,y),.....]
#wall_2 = list(Equidistantpoints(w_st2,w_sp2))



#print("wall1",wall_1)


# 2.  now we will make walls


wall_1_x,wall_1_y = line(w_st1,w_sp1)
ox = ox + wall_1_x
oy = oy +wall_1_y
wall_2_x,wall_2_y = line(w_st2,w_sp2)
ox = ox + wall_2_x
oy = oy +wall_2_y

# 3.  now we will make gate side lines


gate_1_x,gate_1_y = gate_side_lines(gpoint[0],gangle[0])
ox = ox + gate_1_x
oy = oy + gate_1_y

gate_2_x,gate_2_y = gate_side_lines(gpoint[1],gangle[1])
ox = ox + gate_2_x
oy = oy + gate_2_y

gate_3_x,gate_3_y = gate_side_lines(gpoint[2],gangle[2])
ox = ox + gate_3_x
oy = oy + gate_3_y

gate_4_x,gate_4_y = gate_side_lines(gpoint[3],gangle[3])
ox = ox + gate_4_x
oy = oy + gate_4_y

gate_5_x,gate_5_y = gate_side_lines(gpoint[4],gangle[4])
ox = ox + gate_5_x
oy = oy + gate_5_y

gate_6_x,gate_6_y = gate_side_lines(gpoint[5],gangle[5])
ox = ox + gate_6_x
oy = oy + gate_6_y

gate_7_x,gate_7_y = gate_side_lines(gpoint[6],gangle[6])
ox = ox + gate_7_x
oy = oy + gate_7_y

gate_8_x,gate_8_y = gate_side_lines(gpoint[7],gangle[7])
ox = ox + gate_8_x
oy = oy + gate_8_y


print("gpoint",gpoint)
print("gangle",gangle)














plt.figure('Map')
# for x in ox:
# 	for y in oy:
# 		plt.plot(x,y,".k")
plt.plot(ox,oy,"xr")
#plt.plot(0.25,1.25,"xr")
for i in range(8):
	plt.plot(gpoint[i][0],gpoint[i][1],".k")
# plt.plot(1.0,2.0,"xr")
# plt.plot(-1.0,-2.0,"xr")


plt.show()	
