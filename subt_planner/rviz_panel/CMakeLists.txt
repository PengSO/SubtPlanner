cmake_minimum_required(VERSION 2.8.3)
project(subt_planner_rviz_panel)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  planner_msgs
  rviz
  std_srvs
)

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/subt_planner_rviz_panel/subt_planner_control.h
  src/subt_planner_control.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
