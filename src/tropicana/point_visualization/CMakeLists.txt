cmake_minimum_required(VERSION 3.0.2)
project(point_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  vision_msgs
  visualization_msgs
)

# catkin macro
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs vision_msgs visualization_msgs
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(point_visualization_node 
    src/point_visualization.cpp
)

add_dependencies(point_visualization_node
    ${point_visualization_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(point_visualization_node
    ${catkin_LIBRARIES}
)