# 查找src文件夹下的所有源文件
# 并将名称保存到 DIR_Exclusive 变量
aux_source_directory(./src Distance)

#关于opencv
find_package(PCL 1.3 REQUIRED)                          
find_package(OpenCV REQUIRED)                             
                                                       
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})                   
link_directories(${PCL_LIBRARY_DIRS})                   
add_definitions(${PCL_DEFINITIONS})  
# 生成链接库，使用上方的变量
add_library (GetDistance ${Distance})

#将include里的头文件加进上方的链接库中
target_include_directories(GetDistance PUBLIC include)
#opencv链接
target_link_libraries(GetDistance ${PCL_LIBRARIES} ${OpenCV_LIBS})
