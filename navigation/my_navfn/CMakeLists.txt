cmake_minimum_required(VERSION 2.8.3)
project(my_navfn)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS -std=c++11)	

#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 
#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR})
include_directories(/home/zach/catkin_ws/src/andbot_simulator/navigation/alb/include)
include_directories(/home/zach/catkin_ws/src/andbot_simulator/navigation/debug/include)
MESSAGE(STATUS "CPATH dir: $ENV{CPATH}")


message("project source dir: "${PROJECT_SOURCE_DIR})	
#FIND_PACKAGE( OpenCV 2.4.8 REQUIRED )

add_library (my_navfn SHARED src/navfn.cpp src/navfn_no_ros.cpp)
#target_link_libraries(my_navfn
    #${OpenCV_LIBS}   )

message(STATUS "include dir: "${OPENCV_DIRS})	
message(STATUS "include dir: "${PROJECT_ROOT_PATH})	
message(STATUS "include dir: "${CMAKE_INCLUDE_PATH})	
message(STATUS "include dir: "${CMAKE_LIBRARY_PATH})	
message(STATUS "include dir: "${CMAKE_MODULE_PATH})	
MESSAGE(STATUS "HOME dir: $ENV{HOME}")
message(STATUS "include dir: "${CMAKE_BINARY_PATH})	
message(STATUS "include dir: "${CMAKE_BUILD_TYPE})	
message(STATUS "include dir: "${CMAKE_MODULE_PATH})	





