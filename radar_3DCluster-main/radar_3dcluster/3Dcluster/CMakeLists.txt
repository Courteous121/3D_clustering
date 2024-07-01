cmake_minimum_required(VERSION 2.6 FATAL_ERROR)                                    
project(3dclu)                                          
find_package(PCL 1.3 REQUIRED)                          
find_package(OpenCV REQUIRED)                             
                                                       
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})                   
link_directories(${PCL_LIBRARY_DIRS})                   
add_definitions(${PCL_DEFINITIONS})                     
add_executable(3dclu main.cpp down_sampling.cpp ignore_cloud.cpp locate.cpp GetDistance.cpp)                         

target_link_libraries(3dclu ${PCL_LIBRARIES} ${OpenCV_LIBS}) 