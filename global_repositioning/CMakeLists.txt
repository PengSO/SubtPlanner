cmake_minimum_required(VERSION 3.0.2)
project(global_repositioning)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cognitive_graph_model REQUIRED)
find_package(voxel_map REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(
  ${PROJECT_NAME}
  src/orientation_search.cpp
  src/repositioning_bounds.cpp
)

cs_install()
cs_export(LIBRARIES)
