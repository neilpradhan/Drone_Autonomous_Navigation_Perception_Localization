back_up.py
import json
import matplotlib.pyplot as plt
import numpy as np
import math
with open('../course_packages/dd2419_resources/worlds_json/awesome.world.json') as json_file:  
    data = json.load(json_file, 'utf-8')
    #print data["markers"]["position"]


#take walls start and walls end as argument

# let arg1 and arg2 be [x,y] for start and stop
def quadrant_to_grid(arg1):
    x_s=float(arg1[0])
    y_s=float(arg1[1])
    #x_e = stop[0]
    #y_e = stop[1]
    

    # 4 cases for 4 quadrants
    if ((0<=x_s<=10) and (0<=y_s<=10))  :
        grid_x = math.floor(40+(x_s/10)*40)
        grid_y = math.floor(0 +((10-y_s)/10)*40)

    elif((-10<=x_s<=0) and (0<=y_s<=10)) :
        grid_x = math.floor(0+((10+x_s)/10)*40)
        grid_y = math.floor(0 +((10-y_s)/10)*40)

    elif((-10<=x_s<=0) and (-10<=y_s<=0))  :
        grid_x = math.floor(0+((10+x_s)/10)*40)
        grid_y = math.floor(40 +((-y_s)/10)*40)   
       
    elif((0<=x_s<=10) and (-10<=y_s<=0))  :
        grid_x = math.floor(40+(x_s/10)*40)
        grid_y = math.floor(40 +((-y_s)/10)*40) 
    
    grid_x = int(grid_x)
    grid_y = int(grid_y)
    return [grid_x,grid_y]



# start will be [x,y] and similarly end will also be [x.y] most likely e[i=0] and f[i=0] 
def shading_grids(start,stop,grid_map):
    x1 = start[0]
    y1 = start[1]
    x2 = stop[0]
    y2 = stop[1]

    min_x = min(x1,x2)
    max_x = max(x1,x2)
    min_y = min(y1,y2)
    max_y = max(y1,y2)
    
    if ((min_x==max_x) and (min_y!=max_y)):
        for j in range(min_y,max_y+1):
            grid_map[min_x,j]=1 # 1 means occupied   

    elif((min_x!=max_x) and (min_y==max_y)):
        for i  in range(min_x,max_x+1):
            grid_map[i,max_y]=1 # 1 means occupied 
    
    elif((min_x!=max_x) and (min_y!=max_y)):        
        for i in range(min_x,max_x+1):
            for j in range(min_y,max_y+1):
                grid_map[i,j]=1 # 1 means occupied   

    
    return grid_map

a = -1* np.ones(shape =(80,80))# 80 x 80 matrix created 
#print(type(data["gates"][0])) data["gates"] is a list and the other is a dictionary
b=[]
for i in range(len(data["walls"])):
    b.append(data["walls"][i]["plane"]["start"])#list
#print(b)

c=[]
for i in range(len(data["walls"])):
    c.append(data["walls"][i]["plane"]["stop"])#list
#print(c)   

#def value_process(start,stop):
b= np.asarray(b) # 4 x 3
c = np.asarray(c) # 4 x 3
#print(type(b)) nd_array

#print(b)
#print(c)

e = np.delete(b,2,axis=1)# 4x2
f = np.delete(c,2,axis=1)# 4x2


print(e)
print(f)
#print(b) all walls "start"
#print(c)all walls "stop" ##  0-10 means 40 to 80

#print(b[0][0])
#print(e)# 4 x2
#print(e[0])
'''
a1=quadrant_to_grid(e[0])
a2=quadrant_to_grid(f[0])
print(a1)
print(a2)
new_map=shading_grids(a1,a2,a)
#new_map[20,24]=1
print(np.unique(new_map))

print(new_map[60:64,24:28])
'''

print(e.shape[0])

a1=quadrant_to_grid(e[4])
a2=quadrant_to_grid(f[4])
new_map=shading_grids(a1,a2,a)
plt.imshow(new_map)
plt.show()






'''
new_map = a

for i in range(e.shape[0]):
    #print(i) # 0,1,2,3
    
    a1=quadrant_to_grid(e[i])
    a2=quadrant_to_grid(f[i])
    new_map=shading_grids(a1,a2,new_map)
print (new_map)

plt.imshow(new_map)
plt.show()
'''

print(quadrant_to_grid(f[4]))
print(quadrant_to_grid(e[4]))



