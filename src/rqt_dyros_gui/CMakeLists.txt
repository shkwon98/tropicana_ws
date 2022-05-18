cmake_minimum_required(VERSION 3.5.0)
project(rqt_dyros_gui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
)

find_package(Qt5Widgets REQUIRED)

catkin_python_setup()

catkin_package(
# INCLUDE_DIRS ${rqt_dyros_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
#  DEPENDS system_lib
)

set(rqt_dyros_gui_SRCS
  src/rqt_dyros_gui.cpp
)

set(rqt_dyros_gui_HDRS
  include/rqt_dyros_gui/rqt_dyros_gui.h
)

set(rqt_dyros_gui_UIS
  src/rqt_dyros_gui.ui
)

qt5_wrap_cpp(rqt_dyros_gui_MOCS ${rqt_dyros_gui_HDRS})
qt5_wrap_ui(rqt_dyros_gui_UIS_H ${rqt_dyros_gui_UIS})

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${rqt_dyros_gui_SRCS}
  ${rqt_dyros_gui_MOCS}
  ${rqt_dyros_gui_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)
