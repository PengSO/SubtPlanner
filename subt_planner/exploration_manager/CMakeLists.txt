cmake_minimum_required(VERSION 3.0.2)
project(subt_planner)

add_definitions(-std=c++17 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(global_repositioning REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(local_exploration REQUIRED)
find_package(planner_common REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ../visualization/mission_visuals/include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  
)

cs_add_library(
  ${PROJECT_NAME}
  src/exploration_coordinator.cpp
  src/exploration_manager_ros.cpp
  src/exploration_state_nodes.cpp
)

cs_add_executable(subt_planner_node src/exploration_manager_node.cpp)
target_link_libraries(subt_planner_node ${PROJECT_NAME} ${local_exploration_LIBRARIES})

cs_install()
cs_export(LIBRARIES)
