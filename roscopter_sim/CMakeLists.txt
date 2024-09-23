cmake_minimum_required(VERSION 3.8)
project(roscopter_sim)

find_package(gazebo QUIET)
IF(NOT gazebo_FOUND)
  message(STATUS "Gazebo not found. Not building roscopter_sim.")
ELSE()
set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rosflight_msgs
#   rosflight_utils
#   std_msgs
#   nav_msgs
#   geometry_msgs
#   sensor_msgs
#   std_srvs
#   gazebo_ros
#   gazebo_plugins
# )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(roscopter_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(Eigen3 REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
#   CATKIN_DEPENDS roscpp rosflight_msgs rosflight_utils std_msgs nav_msgs geometry_msgs sensor_msgs std_srvs gazebo_ros gazebo_plugins
#   DEPENDS EIGEN3 GAZEBO
# )

include_directories(include 
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

# add_library(multirotor_forces_and_moments_plugin src/multirotor_forces_and_moments.cpp)
# target_link_libraries(multirotor_forces_and_moments_plugin 
#   ${ament_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
#   ${GAZEBO_LIBRARIES}
# )
# # add_dependencies(multirotor_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})
# ament_export_dependencies(
#   multirotor_forces_and_moments_plugin
#   rclcpp
#   rosflight_msgs
#   rosflight_utils
#   std_msgs
#   nav_msgs
#   geometry_msgs
#   sensor_msgs
#   std_srvs
#   gazebo_ros
#   gazebo_plugins
# )

# install(
#   TARGETS
#     multirotor_forces_and_moments_plugin
#   DESTINATION lib/${PROJECT_NAME}
#   # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

add_executable(gazebo_truth_publisher
  src/state_transcription_node.cpp
)
ament_target_dependencies(gazebo_truth_publisher roscopter_msgs rclcpp nav_msgs)
install(TARGETS
        gazebo_truth_publisher
        DESTINATION lib/${PROJECT_NAME})

ament_package()
ENDIF()
