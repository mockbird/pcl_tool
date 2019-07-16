# cmake_minimum_required(VERSION 2.6)
# project(del)
# 
# add_executable(del main.cpp)
# 
# install(TARGETS del RUNTIME DESTINATION bin)
# 


cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(del)

set(CMAKE_BUILD_TYPE = release)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (del main.cpp)
target_link_libraries (del ${PCL_LIBRARIES})

