cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

 project(statistical_removal)
 
 find_package(PCL 1.2 REQUIRED)
 
 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

 cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
 
 project(remove_outliers)
 
 find_package(PCL 1.2 REQUIRED)
 
 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

add_executable (remove_outliers remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})
