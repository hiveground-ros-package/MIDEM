cmake_minimum_required(VERSION 2.8.3)
project(midem_user_interaction)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure sensor_msgs interaction_msgs kinect_msgs message_filters tf pluginlib)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtXml)
include(${QT_USE_FILE})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/dynamic_config.cfg
)

include_directories(
  include
  cfg/cpp
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
)

set(qt_hdrs
  include/midem_user_interaction/midem_user_interaction.h
)

set(qt_srcs	  
  src/main.cpp
  src/midem_user_interaction.cpp
)
 
set(qt_uis
  src/user_interaction.ui
)

qt4_automoc(${qt_srcs})
qt4_wrap_cpp(qt_moc_srcs ${qt_hdrs})
qt4_wrap_ui(uis_h ${qt_uis})  

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  ${qt_srcs}
  ${qt_moc_srcs}
  ${uis_h}
)     

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

add_library(hg_gesture_detector
  src/gesture_detector/rubber_band_hand_gesture_detector.cpp
)






