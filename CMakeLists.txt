# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(pcl)

####################### opencv #####################
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


######################## pcl ########################
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 这里括号里面的两个参数分别是工程项目名和我们要编译文件名的意思，记住中间一空格键隔开
add_executable(pcl main.cpp)

target_link_libraries(pcl ${PCL_LIBRARIES} ${OpenCV_LIBS})

set(CMAKE_BUILD_TYPE Debug) 
