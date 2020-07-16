cmake_minimum_required(VERSION 3.0.2)
project(slam_study)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  #include
)
add_executable(img_print_node src/img_print.cpp)
add_executable(tracking_node src/orb_matching.cpp)

target_link_libraries(img_print_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

target_link_libraries(tracking_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)
