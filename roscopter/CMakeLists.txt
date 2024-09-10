set(CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 3.8)
project(roscopter)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(inertial_sense QUIET)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(roscopter_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if (${inertial_sense_FOUND})
    message(WARNING "found inertial sense library")
    add_definitions(-DINERTIAL_SENSE)
else()
    message(WARNING "Did NOT find inertial sense library")
endif()

find_package(ublox_read_2 QUIET)

if (${ublox_read_2_FOUND})
    message(WARNING "found ublox_read_2 library")
    add_definitions(-DUBLOX)
else()
    message(WARNING "Did NOT find ublox_read_2 library")
endif()

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_dependencies(rclcpp rclpy roscopter_msgs rosflight_msgs std_msgs nav_msgs sensor_msgs EIGEN3 geometry_msgs)

add_definitions(-DROSCOPTER_DIR="${CMAKE_CURRENT_LIST_DIR}")

# Included from ROSPlane
install(
  DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME}/)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS} 
  ${ament_INCLUDE_DIRS}
  )


### LIBRARIES ###

# Param Manager
add_library(param_manager
  include/param_manager/param_manager.hpp
  src/param_manager/param_manager.cpp
)
ament_target_dependencies(param_manager rclcpp)
ament_export_targets(param_manager HAS_LIBRARY_TARGET)
install(DIRECTORY include/param_manager DESTINATION include)
install(TARGETS param_manager
  EXPORT param_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(ekf
  src/ekf/state.cpp
  src/ekf/dynamics.cpp
  src/ekf/ekf.cpp
  src/ekf/meas.cpp
  src/ekf/log.cpp)
target_include_directories(ekf PUBLIC include lib/geometry/include)
target_link_libraries(ekf ${YAML_CPP_LIBRARIES} stdc++fs)

add_library(ekf_ros src/ekf/ekf_ros.cpp)
target_link_libraries(ekf_ros ekf ${ament_LIBRARIES})
ament_target_dependencies(ekf_ros rclcpp rclpy roscopter_msgs rosflight_msgs std_msgs nav_msgs sensor_msgs EIGEN3 geometry_msgs)

add_executable(ekf_node src/ekf/ekf_node.cpp)
target_link_libraries(ekf_node ekf_ros ${ament_LIBRARIES})
ament_target_dependencies(ekf_node rclcpp rclpy roscopter_msgs rosflight_msgs std_msgs nav_msgs sensor_msgs EIGEN3 geometry_msgs)
install(
  TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME})

# External attitude transcriber
add_executable(ext_att_transcriber
  src/ekf/external_attitude_transcriber.cpp)
ament_target_dependencies(ext_att_transcriber rclcpp roscopter_msgs rosflight_msgs)
target_link_libraries(ext_att_transcriber ${ament_LIBRARIES})
install(TARGETS ext_att_transcriber
  DESTINATION lib/${PROJECT_NAME})



### EXECUTABLES ###

# Controller
add_executable(controller
  src/controller/controller_ros.cpp
  src/controller/controller_state_machine.cpp
  src/controller/controller_cascading_pid.cpp
  src/controller/simple_pid.cpp)
ament_target_dependencies(controller rclcpp rclpy roscopter_msgs rosflight_msgs)
target_link_libraries(controller ${ament_LIBRARIES} param_manager)
install(
  TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

# Trajectory Follower
add_executable(trajectory_follower
  src/navigation/trajectory_follower_ros.cpp
  src/navigation/trajectory_follower.cpp
  src/controller/simple_pid.cpp)
ament_target_dependencies(trajectory_follower rclcpp rclpy roscopter_msgs)
target_link_libraries(trajectory_follower ${ament_LIBRARIES} param_manager)
install(
  TARGETS trajectory_follower
  DESTINATION lib/${PROJECT_NAME})

# Path Manager
add_executable(path_manager
  src/navigation/path_manager_ros.cpp
  src/navigation/path_manager.cpp)
ament_target_dependencies(path_manager rclcpp rclpy roscopter_msgs std_srvs)
target_link_libraries(path_manager ${ament_LIBRARIES} param_manager)
install(
  TARGETS path_manager
  DESTINATION lib/${PROJECT_NAME})
  
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
