# cmake_minimum_required(VERSION 2.6)
# project(registration_ndt)
# 
# add_executable(registration_ndt main.cpp)
# 
# install(TARGETS registration_ndt RUNTIME DESTINATION bin)



cmake_minimum_required(VERSION 2.8 FATAL_ERROR) #对于cmake版本的最低版本的要求

project(registration_ndt) #建立的工程名
SET(CMAKE_BUILD_TYPE Release)

add_definitions(-std=c++11)
find_package(PCL 1.2 REQUIRED) #要求工程依赖的PCL最低版本

#下面的语句是利用CMake的宏完成对PCL的头文件路径和链接路径变量的配置和添加，如果缺少下面几行，生成文件的过程中就会提示
#找不到相关的头文件，在配置CMake时，当找到了安装的PCL，下面相关的包含的头文件，链接库，路径变量就会自动设置
#                    PCL_FOUND:如果找到了就会被设置为1 ，否则就不设置
#                    PCL_INCLUDE_DIRS:被设置为PCL安装的头文件和依赖头文件的目录
#                    PCL_LIBRARIES:被设置成所建立和安装的PCL库头文件
#                    PCL_LIBRARIES_DIRS:被设置成PCL库和第三方依赖的头文件所在的目录
#                    PCL_VERSION:所找到的PCL的版本
#                    PCL_COMPONENTS:列出所有可用的组件
#                    PCL_DEFINITIONS:列出所需要的预处理器定义和编译器标志
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#这句话告诉CMake从单个源文件建立一个可执行文件
add_executable (registration_ndt main.cpp)
#虽然包含了PCL的头文件，因此编译器知道我们现在访问所用的方法，我们也需要让链接器知道所链接的库，PCL找到库文件由
#PCL_COMMON_LIBRARIES变量指示，通过target_link_libraries这个宏来出发链接操作
target_link_libraries (registration_ndt ${PCL_LIBRARIES})

find_package(Eigen3)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})