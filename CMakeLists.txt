cmake_minimum_required(VERSION 2.8.3)
project(ds_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ds_core_msgs
  ds_actuator_msgs
  ds_sensor_msgs
  roscpp
  sensor_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("include")

# Find the protobuf directories for gazebo
find_package(Protobuf REQUIRED QUIET)
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# Build any protobuf messages we're going to use
set (gazebo_msgs
  gazebo_msgs/SensorDepth.proto
  gazebo_msgs/SensorDvl.proto
  gazebo_msgs/SensorIns.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${gazebo_msgs})
add_library(ds_sim_gazebo_msgs SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(ds_sim_gazebo_msgs ${PROTOBUF_LIBRARY})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# dsros_jointstate_publisher
add_library(dsros_jointstate_publisher src/dsros_jointstate_publisher.cc)
target_link_libraries(dsros_jointstate_publisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dsros_jointstate_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(SENSORS_SRCS
    gazebo_src/depth_util.cc       gazebo_src/depth_util.hpp
    gazebo_src/dsros_depth.cc      gazebo_src/dsros_depth.hh
    gazebo_src/dsros_ins.cc        gazebo_src/dsros_ins.hh
    gazebo_src/dsros_dvl.cc        gazebo_src/dsros_dvl.hh
    gazebo_src/dsros_sensors.cc)

# dsros_sensors (passed at STARTUP as a system plugin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
add_library(dsros_sensors ${SENSORS_SRCS})
target_link_libraries(dsros_sensors ds_sim_gazebo_msgs ${GAZEBO_LIBRARIES})
add_dependencies(dsros_sensors ds_sim_gazebo_msgs)

# dsros_ros_depth
add_library(dsros_ros_depth src/dsros_depth_plugin.cc src/dsros_depth_plugin.hh)
target_link_libraries(dsros_ros_depth dsros_sensors ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dsros_ros_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# dsros_ros_ins
add_library(dsros_ros_ins src/dsros_ins_plugin.cc src/dsros_ins_plugin.hh)
target_link_libraries(dsros_ros_ins dsros_sensors ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dsros_ros_ins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# DVL plugin
add_library(dsros_ros_dvl src/dsros_dvl_plugin.cc src/dsros_dvl_plugin.hh)
target_link_libraries(dsros_ros_dvl dsros_sensors ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dsros_ros_dvl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# GPS plugin
add_library(dsros_ros_gps src/dsros_gps_plugin.cc src/dsros_gps_plugin.hh)
target_link_libraries(dsros_ros_gps ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dsros_ros_gps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ds_sim_gazebo_msgs  dsros_jointstate_publisher dsros_sensors dsros_ros_depth dsros_ros_ins dsros_ros_dvl dsros_ros_gps
  CATKIN_DEPENDS xacro roscpp gazebo_ros sensor_msgs ds_core_msgs ds_sensor_msgs ds_actuator_msgs
  DEPENDS protobuf-compiler
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


