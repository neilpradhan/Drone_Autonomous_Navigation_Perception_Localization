# Drone_Autonomous_Navigation_Perception_Motion_planning
dd2419 project course on drones

most important files :
1.grid.py
2.A_star_build_path.py
3.Fly_drone.py


# 2d_pose_graph_SLAM

## Table of contents
* [Abstract](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)

## Abstract:
Building a good map of an environment has been a long prevalent
problem in the domain of robotics. Especially in areas where there
is no access to external referencing systems such as GPS sensors.
This problem is popularly known as Simultaneous localization and
mapping problem.
In this project, I have attempted to study, code and demonstrate
the working of 2D-pose Graph Simultaneous localization and map-
ping Optimization on the data sets that are provided by Luca Car-
lone, Department of Aeronautics and Astronautics, MIT. 
Moreover, I have not used any external exclusive Graph Slam
library for implementation. Given that pose graph SLAM can be
divided into two functional modules namely the front-end which
comprises converting raw data into graph nodes and edges and
the back-end which takes the nodes and edges as input to find 
the optimized poses, this project will only address the back-end
optimization function.

To see the complete report: [2D pose graph Slam Project report](https://github.com/neilpradhan/2d_pose_graph_SLAM/blob/master/Applied_Estimation_Graph_Slam_Project_Report.pdf)
	
## Technologies
Project is created with:
* Python 3
* Scipy
* Pandas
* Tensorflow
	
## Setup
To run this project, go to https://lucacarlone.mit.edu/datasets/ download the datasets for 2d pose graph SLAM or if you are using your own datasets, make sure you have them in the same format. You can always change the "get_from_dataset" function to have it in your own format

```
$ cd 2d_pose_graph_SLAM/final_project
$ python script.py
$ python pose_graph_two_d.py
```


