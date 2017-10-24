cmake_minimum_required(VERSION 2.8.3)
project(dsl_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dsl_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dsl_sim
  CATKIN_DEPENDS gazebo xacro dsl_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


