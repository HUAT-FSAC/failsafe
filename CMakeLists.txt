cmake_minimum_required(VERSION 3.0.2)
project(failsafe)

add_compile_options(-std=c++11)

find_package(
    catkin REQUIRED
    roscpp
    sensor_msgs
    geometry_msgs
    common_msgs
)


catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs common_msgs
)

include_directories(
    /home/neko/Code/dev_ws/devel/include
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    failsafe_node src/main.cpp
)

target_link_libraries(
    failsafe_node
    ${catkin_LIBRARIES}
)
