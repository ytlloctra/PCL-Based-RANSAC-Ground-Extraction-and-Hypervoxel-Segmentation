cmake_minimum_required(VERSION 3.22)

#项目名称
project(background_remove)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#库文件名称  name.cpp
add_executable (background remove.cpp)
target_link_libraries (background ${PCL_LIBRARIES})
