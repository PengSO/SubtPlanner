cmake_minimum_required(VERSION 3.0.2)
project(local_exploration)

add_definitions(-std=c++17 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(global_repositioning REQUIRED)
find_package(planner_common REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ../subt_planner/visualization/mission_visuals/include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME}
  src/local_exploration_planner.cpp
  src/local_exploration_graph.cpp
  src/local_exploration_sampling.cpp
  src/local_exploration_integration.cpp
  ../subt_planner/visualization/mission_visuals/src/mission_visuals.cpp
)

cs_install()
cs_export(LIBRARIES)
