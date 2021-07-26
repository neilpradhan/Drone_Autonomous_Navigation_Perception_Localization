# Drone_Autonomous_Navigation_Perception_Localization

![alt text](https://github.com/neilpradhan/Drone_Autonomous_Navigation_Perception_Motion_planning/blob/test_branch/images/CFwithCam.jpg)
---

<!-- <details open="open"> -->
<summary>Table of Contents</summary>

- [About](#about)
- [Final video](#video)
- [Tools](#tools)
- [Perception](#perception)
  - [Data collection and Extraction](#data-collection-and-extraction)
  - [Data Annotation](#data-annotation)
  - [Training and Evaluation](#training-and-evaluation)
- [Navigation and Localization](#navigation-and-localization)

<!-- - [About](#about)
- [Final video](#video)
- [Tools](#tools)
- [Perception](#perception)
    -[Data collection and extraction](#data-collection-and-extraction)
    -[Data Annotation](#data-annotation)
    -[Training and Evaluation] (#training-and-evaluation)
- [Navigation and Localization] (#navigation-and-localization) -->
	
	

<!-- </details> -->

---




## About:

<!-- To see the complete report: [2D pose graph Slam Project report](https://github.com/neilpradhan/2d_pose_graph_SLAM/blob/master/Applied_Estimation_Graph_Slam_Project_Report.pdf) -->

In this project, a research purpose drone consists of the following sensors: camera, Inertial Measurement Unit and flow deck. The task in this project is that we have to implement complete autonomous navigation (static obstacle avoidance) and motion planning as well as time traffic sign classification, detection and pose/depth estimation all in real time conditions. The localization is aruco-marker based where the map cordinates for the aruco marker poses (x,y,z, yaw pitch roll) will be provided alogn with that there are a couple of gates that are required to pass through autonomously, the cordinates of which were provided in the map as a json file.

> Note: Here, in this repo, I only share files which are created and owned by me and my team members, the setup requires crazyflie_client ROS files which can be obtained from the drone manufacturer [bitcraze](https://www.bitcraze.io/products/old-products/crazyflie-2-0/) who has the authority over their code distribution. Inorder to run this project we have to BUILD using CATKIN MAKE all the modules created by us as well as those provided by the manufacturers
	
## Final video
 To see the complete project in action, please click on this link [Drone Perception, Localization and Navigation](https://www.youtube.com/watch?v=zHv-CBUqLFw&t=5s)
	
## Tools
Project is created with:
* Python 3, C++
* ROS Kinetic
* Machine learning and Deep learning libraries
* Computer vision libraries
* CMAKE

## Perception
![alt text](https://github.com/neilpradhan/Drone_Autonomous_Navigation_Perception_Motion_planning/blob/test_branch/images/yolo_snap.PNG)
In this task we have to use the drone camera inorder to detect traffic sings all over the map and  estimate in real time their distances (poses) from the drone. Real time object detection and pose estimation using camera parameters and drone localization information from aruco-marker detection.

### Data collection and extraction
The drone camera is moved over the traffic sings and data is collected using ROS BAG files.This data is used to extract images which are further used to extract frames and 1000 images containing 15 swedish traffic sings are created.

### Data Annotation
Open source software YAT is used for data annotation and ground truth labelling

### Training and Evaluation
YOLO V2 is used for training and later evaluation is done to see if the pose\DEPTH estimation is within 1 metre of error according to the requirement. Further optimization resuls in pose estimation error less than 0.3 m

## Navigation and Localization
![alt text](https://github.com/neilpradhan/Drone_Autonomous_Navigation_Perception_Motion_planning/blob/test_branch/images/Image%20from%20iOS.jpg)
![alt text](https://github.com/neilpradhan/Drone_Autonomous_Navigation_Perception_Motion_planning/blob/test_branch/images/final_path.png)
In this part, we have written scripts to take the map cordinates and obstacle co-ordinates and plan the path using A- star Graph theory methods. For the localization sevral techniques were used to find the best transform, which has the most minimum error.For motion planning approximate errors occured due to localization were compensated

