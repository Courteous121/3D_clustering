find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED) 
                                           
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS})                   
add_definitions(${PCL_DEFINITIONS})  

# ==== Libraries ====
aux_source_directory(./src DIR_LIDAR)

add_library (Lidar ${DIR_LIDAR})
target_include_directories(Lidar PUBLIC include)

target_link_libraries(Lidar
  PUBLIC Eigen3::Eigen
  PUBLIC jsoncpp_lib
  ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Lidar
  PUBLIC Threads::Threads glfw GLEW::GLEW OpenGL::GL )

# target_link_libraries(Lidar ${PCL_LIBRARIES} ${OpenCV_LIBS})






