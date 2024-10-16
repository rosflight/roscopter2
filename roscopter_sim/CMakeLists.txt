cmake_minimum_required(VERSION 3.8)
project(roscopter_sim)

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
set_source_files_properties(geomag.c PROPERTIES LANGUAGE C)

add_executable(mocap_translation
  src/mocap_translation_node.cpp
)
ament_target_dependencies(mocap_translation roscopter_msgs rclcpp nav_msgs geometry_msgs sensor_msgs)
install(TARGETS
        mocap_translation
        DESTINATION lib/${PROJECT_NAME})

add_executable(mocap_gnss
  src/mocap_gnss.cpp
)
ament_target_dependencies(mocap_gnss roscopter_msgs rclcpp nav_msgs geometry_msgs sensor_msgs rosflight_msgs Eigen3)
target_include_directories(mocap_gnss PUBLIC include)
install(TARGETS
        mocap_gnss
        DESTINATION lib/${PROJECT_NAME})

add_executable(mocap_mag
  src/mocap_magnetometer.cpp
  src/utils/geomag.c
)
ament_target_dependencies(mocap_mag roscopter_msgs rclcpp nav_msgs geometry_msgs sensor_msgs rosflight_msgs Eigen3)
target_include_directories(mocap_mag PUBLIC include)
install(TARGETS
        mocap_mag
        DESTINATION lib/${PROJECT_NAME})

ament_package()
