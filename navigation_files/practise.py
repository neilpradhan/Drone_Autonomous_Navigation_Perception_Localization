import json
import matplotlib.pyplot as plt
import numpy as np
import math
with open('../course_packages/dd2419_resources/worlds_json/tutorial_1.world.json') as json_file:  
    data = json.load(json_file, 'utf-8')



g=[]
for i in range(len(data["gates"])):	
    g.append(data["gates"][i]["position"])	
print(g)

g = np.asarray(g)# gates
print(np.shape(g)) 

g1 = np.delete(g,2,axis=1)# 4x2

print(g1) # 4x2  

## manual tweeking for now can be more robust based on the heading angles
#gate1
st_1 = [g1[0][0],g1[0][1]+0.2]
ed_1 = [g1[0][0],g1[0][1]-0.2]

st_2 = [g1[1][0]-0.2,g1[1][1]]
ed_2 = [g1[1][0]+0.2,g1[1][1]]

st_3 = [g1[2][0],g1[2][1]-0.2]
ed_3 = [g1[2][0],g1[2][1]+0.2]

st_4 = [g1[3][0]-0.2,g1[2][1]]
ed_4 = [g1[3][0]+0.2,g1[2][1]]



print(st_1)



