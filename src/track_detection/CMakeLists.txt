cmake_minimum_required(VERSION 2.8.3)
project(track_detection)

set(OpenCV_DIR /usr/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV
  cv_bridge
  image_transport
  geometry_msgs
  sensor_msgs
)


catkin_package(CATKIN_DEPENDS std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(
    INCLUDE_DIRS include
    include
    include/${PROJECT_NAME}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(Track_Detection_node src/Track_Detection.cpp)
target_link_libraries(Track_Detection_node 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)


