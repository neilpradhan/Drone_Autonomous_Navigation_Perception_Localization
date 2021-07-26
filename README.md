# Drone_Autonomous_Navigation_Perception_Motion_planning




<details open="open">
<summary>Table of Contents</summary>

<!-- - [About](#about)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Usage](#usage)
    - [Cookiecutter template](#cookiecutter-template)
    - [Manual setup](#manual-setup)
    - [Variables reference](#variables-reference)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [Support](#support)
- [License](#license)
- [Acknowledgements](#acknowledgements) -->

- [About](#about)
- [Final video](#video)
- [Tools](#tools)
- [Perception](#perception)
  -[Data collection and extraction](#data_preparation)
  -[Data Annotation](#data_annotation)
- [Navigation and Localization] (#navigation_and_localization)
	
	

</details>

---




## About:

<!-- To see the complete report: [2D pose graph Slam Project report](https://github.com/neilpradhan/2d_pose_graph_SLAM/blob/master/Applied_Estimation_Graph_Slam_Project_Report.pdf) -->

In this project, a research purpose drone consists of the following sensors: camera, Inertial Measurement Unit and flow deck. The task in this project is that we have to implement complete autonomous navigation (static obstacle avoidance) and motion planning as well as time traffic sign classification, detection and pose/depth estimation all in real time conditions. The localization is aruco-marker based where the map cordinates for the aruco marker poses (x,y,z, yaw pitch roll) will be provided alogn with that there are a couple of gates that are required to pass through autonomously, the cordinates of which were provided in the map as a json file.
	
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
In this task we have to use the drone camera inorder to detect traffic sings all over the map and  estimate in real time their distances (poses) from the drone. Real time object detection and pose estimation using camera parameters and drone localization information from aruco-marker detection.

### Data collection and extraction
The data is collected as a rosbag files from which we get the videos initially and later collect and save the frames.


## Navigation and Localization

