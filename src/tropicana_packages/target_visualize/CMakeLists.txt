cmake_minimum_required(VERSION 3.0.2)
project(target_visualize)

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
    ../tropicana_activate/include
    ${catkin_INCLUDE_DIRS}
)


add_executable(target_visualize_node 
    src/target_visualize.cpp
)

add_dependencies(target_visualize_node
    ${target_visualize_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(target_visualize_node
    ${catkin_LIBRARIES}
)