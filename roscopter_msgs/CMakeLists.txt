cmake_minimum_required(VERSION 3.8)
project(roscopter_msgs)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

### SET MSG AND SRV FILE DEFINITIONS ### 

set(msg_files
  "msg/ControllerCommand.msg"
  "msg/ControllerInternals.msg"
  "msg/PoseEuler.msg"
  "msg/Bool.msg"
  "msg/State.msg"
  "msg/TrajectoryCommand.msg"
  "msg/Waypoint.msg"
)

set(srv_files
  "srv/AddWaypoint.srv"
  "srv/AddWaypointList.srv"
  "srv/RemoveWaypoint.srv"
  "srv/SetWaypointsFromFile.srv"
  "srv/ListWaypoints.srv"
  "srv/ClearWaypoints.srv"
  "srv/Hold.srv"
  "srv/Release.srv"
  "srv/Land.srv"
  "srv/Fly.srv"
  "srv/ReturnToBase.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  std_msgs
  )

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


ament_export_dependencies(rosidl_default_runtime)

ament_package()