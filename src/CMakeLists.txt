cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(main)

find_package(PCL 1.12 CONFIG REQUIRED COMPONENTS io registration segmentation visualization)
find_package(realsense2 CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp)
target_link_libraries (main ${PCL_LIBRARIES} ${realsense2_LIBRARY} ${OpenCV_LIBS})
