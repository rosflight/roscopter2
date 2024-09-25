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

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(yaml-cpp REQUIRED)
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
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

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
  ${YAML_CPP_INCLUDEDIR}
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

set_source_files_properties(geomag.c PROPERTIES LANGUAGE C)
# Estimator
add_executable(roscopter_estimator
              src/ekf/estimator_node.cpp
              src/ekf/estimator_ros.cpp
              src/ekf/estimator_ekf.cpp
              src/ekf/estimator_continuous_discrete.cpp
              src/utils/geomag.c)
target_link_libraries(roscopter_estimator
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(roscopter_estimator roscopter_msgs rosflight_msgs sensor_msgs rclcpp Eigen3)
target_link_libraries(roscopter_estimator param_manager)
target_include_directories(roscopter_estimator PUBLIC include)
install(TARGETS
  roscopter_estimator
  DESTINATION lib/${PROJECT_NAME})

add_library(estimator_lib src/ekf/estimator_node.cpp src/ekf/estimator_ros.cpp src/ekf/estimator_ekf.cpp src/ekf/estimator_continuous_discrete.cpp src/utils/geomag.c include/ekf/estimator_ros.hpp include/ekf/estimator_ekf.hpp include/ekf/estimator_continuous_discrete.hpp)
target_link_libraries(roscopter_estimator estimator_lib)
target_link_libraries(estimator_lib param_manager)
ament_target_dependencies(estimator_lib rclcpp geometry_msgs rosflight_msgs sensor_msgs Eigen3 roscopter_msgs)

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

# Path Planner
add_executable(path_planner
  src/navigation/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp roscopter_msgs std_srvs rosflight_msgs)
target_link_libraries(path_planner ${ament_LIBRARIES} param_manager ${YAML_CPP_LIBRARIES})
install(
  TARGETS path_planner
  DESTINATION lib/${PROJECT_NAME}) 
  
if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(estimator_test test/estimator_test.cpp)
  ament_target_dependencies(estimator_test rclcpp geometry_msgs rosflight_msgs sensor_msgs Eigen3 roscopter_msgs)
  target_include_directories(estimator_test PUBLIC
    include
  )
  target_link_libraries(estimator_test estimator_lib) 

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
