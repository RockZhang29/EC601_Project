# EC601_Project
This repository is for EC601 Group 1 Project: Detecting 3D Point Clouds: Exploring Unknown Environments

The project contributors are Zhengyi Cao, Gianna Iafrate, Tianshi Zhang, and Jinzhou Zhao of Boston University's College of Engineering Masters of Science in Computer & Electrical Engineering Program. 

This repository contains all the files and data from the project. For anything not in this Github due to file size, it is available in:

Google Drive link - extra files and sprint presentations:
https://drive.google.com/drive/u/1/folders/1jKGzOmRHx0YQrErTRjcG8uH9SL3HK8vk

## Organization of GitHub Repo:

###### A-LOAM
Files and code necessary to generate the map from recorded data
###### DataAndPhotos
Containing resulting photos of map, registration, filtering, and path-planning
###### FilterResearch
Research performed prior to finalization of code
###### Jackal_ws
Zip files containing information on the robot, ros, and workspace
###### PathPlanningFinal
Final path planning files and data used
###### PathPlanningResearch
Research performed prior to finalization of code
###### PointCloudResearch
General research on parsing point-cloud data
###### Registration
Code and files necessary to run registration on data
###### UsefulResources
Any resources and articles utilized to learn more about the topic

## Project Overview:
- Create a map of an unknown area in advance to facilitate further exploration
- Collect information about the area, including position and volume, as well as obstacles within the area
- Generate 3D point map of random area and plan trajectory through the area

## Implementation Structure:
The following lists the implementation structure. Listed next to each step will be the tools and software necessary to complete:
1. Utilize robot with LiDAR to move through environment (Velodyne VLP-16 3D LiDAR)
2. Record data and generate map (ALOAM Algorithm and ROS (C++))
3. Registration of data from multiple files into one framewor (MATLAB)
4. Filtering out noise, outliers, and redundant data (Point Cloud Library, C++)
5. Categorize obstacles and free space (Python)
6. Plan trajectory through environment (Python, Rapidly-Explorng Random Tree (RRT) Path Planning Algorithm)

## Conclusions & Future Research:
In conclusion, we were able to successfully generate a point cloud map of the environment that acn represent obstacles and corners. The path planned can maneuver around the presented features as well. However, the LiDAR is not sensitive enough to pick up all the data, and therefore the path planning trajectory is not the optimal path. In general, the map has some bumps and room for improvement, as well as improving the path-planning algorithm.

In the future, one may want to use a more advanced LOAM to improve the quality of the map. In addition, AI can be used to obtain better point cloud registration. Path-planning in real-time is the goal, and being able to do that while generating the map is the next step, as well as being able to navigate narrow passageways. A remaining future step would be to attempt this method on various other LiDAR vehicles to increase the flexibility and reusability of the method. 
