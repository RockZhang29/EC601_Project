# This README shows how to use one main filter used in the project
The filter part is done in Ubuntu 18.04 with ROS melodic installed, besides, you need to install pcl tools to visualize the .pcd files

So, do the following command in Linux terminal:

`sodu apt-get install pcl_tools`
### StatisticOutlierRemoval Filter
1. Creat a workspace folder, such as `statistic_ws`; 
2. Download the c++ file named `statistical_removal.cpp` into folder `statistic_ws`;
3. Create the CMakelists file in  folder `statistic_ws`, named `CMakeLists.txt`;
4. Copy following content into `CMakeLists.txt` and save:

`cmake_minimum_required(VERSION 3.5 FATAL_ERROR)`

 `project(statistical_removal)`
 
 `find_package(PCL 1.2 REQUIRED)`
 
 `include_directories(${PCL_INCLUDE_DIRS})`
 `link_directories(${PCL_LIBRARY_DIRS})`
 `add_definitions(${PCL_DEFINITIONS})`

`add_executable (statistical_removal statistical_removal.cpp)`
`target_link_libraries (statistical_removal ${PCL_LIBRARIES})`

 `cmake_minimum_required(VERSION 3.5 FATAL_ERROR)`
 
 `project(remove_outliers)`
 
 `find_package(PCL 1.2 REQUIRED)`
 
 `include_directories(${PCL_INCLUDE_DIRS})`
 `link_directories(${PCL_LIBRARY_DIRS})`
 `add_definitions(${PCL_DEFINITIONS})`

`add_executable (remove_outliers remove_outliers.cpp)`
`target_link_libraries (remove_outliers ${PCL_LIBRARIES})`

5. Create a folder named `build` in folder `statistic_ws`;
6. Open terminal in `build`, and run command: 

`cmake ..`

7. Run command:

`make`

8. Download `object3d.pcd` file into folder `build`
9. Run command:

`./statistic_removal`
