cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Interface generation must precede the library target (tutorial §6.4 #10).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleState.msg"
  "msg/ObstacleStateArray.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

set(mpc_controller_dependencies
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# rosidl_generate_interfaces already owns the "mpc_controller" CMake target,
# so the plugin library target is renamed while keeping libmpc_controller.so
# (plugin XML path must match the output library name).
add_library(mpc_controller_core SHARED src/mpc_controller.cpp)
set_target_properties(mpc_controller_core PROPERTIES OUTPUT_NAME mpc_controller)
target_include_directories(mpc_controller_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(mpc_controller_core ${mpc_controller_dependencies})

# The plugin consumes the same-package messages (D3 heartbeat subscription).
rosidl_get_typesupport_target(
  mpc_controller_typesupport_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mpc_controller_core "${mpc_controller_typesupport_cpp}")

# Both registration sites are required by Nav2/pluginlib in Humble.
pluginlib_export_plugin_description_file(nav2_core mpc_controller_plugin.xml)

# W5 D4: vendored OSQP 0.6.3 (pin details in third_party/VENDORED.md).
# EXCLUDE_FROM_ALL keeps OSQP's own install rules out of this package.
set(UNITTESTS OFF CACHE BOOL "" FORCE)
set(PRINTING OFF CACHE BOOL "" FORCE)
set(PROFILING ON CACHE BOOL "" FORCE)
set(CTRLC OFF CACHE BOOL "" FORCE)
set(ENABLE_MKL_PARDISO OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/osqp EXCLUDE_FROM_ALL)

add_executable(osqp_smoke src/osqp_smoke.cpp)
target_compile_features(osqp_smoke PRIVATE cxx_std_17)
target_link_libraries(osqp_smoke PRIVATE osqpstatic m rt dl)

install(TARGETS mpc_controller_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/fake_sensors.py
  scripts/mock_obstacle_heartbeat.py
  scripts/smoke_check.py
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS osqp_smoke RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(mpc_controller_core)
ament_export_dependencies(${mpc_controller_dependencies})

ament_package()
