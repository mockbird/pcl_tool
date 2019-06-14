# cmake_minimum_required(VERSION 2.6)
# project(global_registration)
# 
# add_executable(global_registration main.cpp)
# 
# install(TARGETS global_registration RUNTIME DESTINATION bin)
# 
# 


cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(global_registration)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (global_registration main.cpp pair_registration.h)
target_link_libraries (global_registration ${PCL_LIBRARIES})