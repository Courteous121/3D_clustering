cmake_minimum_required (VERSION 3.10)
project (3dclu)

set(CMAKE_BUILD_TYPE Debug)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
find_package(PCL 1.3 REQUIRED)                          
find_package(OpenCV REQUIRED)                             
                                                       
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})                   
link_directories(${PCL_LIBRARY_DIRS})                   
add_definitions(${PCL_DEFINITIONS})                     
add_executable(3dclu main.cpp)
add_subdirectory(Modules)
target_link_libraries(3dclu down_sampling)
target_link_libraries(3dclu GetDistance)
target_link_libraries(3dclu ignore_cloud)
target_link_libraries(3dclu locate)
target_link_libraries(3dclu Lidar)



target_link_libraries(3dclu ${OpenCV_LIBS})
target_link_libraries(3dclu ${PCL_LIBRARIES} ${OpenCV_LIBS}) 


find_package(PCL 1.3 REQUIRED)                          
find_package(OpenCV REQUIRED)                             
                                                       
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})                   
link_directories(${PCL_LIBRARY_DIRS})                   
add_definitions(${PCL_DEFINITIONS})
# set(CMAKE_CXX_STANDARD 14)

# add_executable(Radar main.cpp)
# add_subdirectory(Modules)
# target_link_libraries(Radar Exclusive)
# target_link_libraries(Radar MOT)
# target_link_libraries(Radar TensorRT)
# target_link_libraries(Radar ${OpenCV_LIBS})