This branch is for Tianshi Zhang and Jinzhou Zhao ME570 final project. 

The Jackal workspace is the basic, important and necssary ros node we using in our project. You can download those workspace from offical website.

The src is the map exploration method, Advanced Lidar Odometry and Mapping workspace. If you want use this, create a new workspace and download the A-LOAM file directly. 

What is more, there is also a video to show our result. And our final Paper is included.

First, we use the Jackal to explore a unknown map directly with the helping of camera. Through camera we are able to see the surrounding envoriment. What is more, we record the whole process with rosbag. After that, ALOAM is able to rebuild the map with point cloud and generating a group of PCD file. 

Second, we are supposed to merge a lot of PCD file till we are able to obtain a complete map. This is point cloud registration.

Last, after obtaining the complete map, we need to filter the point cloud, such as remove unreasonable point, clear the surface, corner and feature, etc. Therefore, we will have a clear and clean completely map.
