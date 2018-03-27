cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITION})

add_executable(icp icp.cpp)
target_link_libraries(icp ${PCL_LIBRARIES})
