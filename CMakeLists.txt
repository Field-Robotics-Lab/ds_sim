cmake_minimum_required(VERSION 2.8.3)
project(ds_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ds_msgs
  roscpp
  sensor_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories("include")


# dsros_jointstate_publisher
add_library(dsros_jointstate_publisher src/dsros_jointstate_publisher.cc)
target_link_libraries(dsros_jointstate_publisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dsros_jointstate_publisher ds_msgs_generate_messages_cpp)

# dsros_sensors (passed at STARTUP as a system plugin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
add_library(dsros_sensors src/dsros_sensors.cc)
target_link_libraries(dsros_sensors ${GAZEBO_LIBRARIES})
add_dependencies(dsros_sensors ds_msgs_generate_messages_cpp)

# dsros_servo
#add_library(dsros_servo src/dsros_servo.cc)
#target_link_libraries(dsros_servo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(dsros_servo ds_msgs_generate_messages_cpp)

# dsros_thruster
#add_library(dsros_thruster src/dsros_thruster.cc)
#target_link_libraries(dsros_thruster ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(dsros_thruster ds_msgs_generate_messages_cpp)

# dsros_dropweights
#add_library(dsros_dropweights src/dsros_dropweights.cc)
#target_link_libraries(dsros_dropweights ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(dsros_dropweights ds_msgs_generate_messages_cpp)

#di sros_depthsensor
#add_library(dsros_depthsensor src/dsros_depthsensor.cc)
#target_link_libraries(dsros_depthsensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(dsros_depthsensor ds_msgs_generate_messages_cpp)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ds_sim
  CATKIN_DEPENDS xacro ds_msgs roscpp gazebo_ros sensor_msgs
#  DEPENDS system_lib
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


