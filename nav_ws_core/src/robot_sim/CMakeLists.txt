cmake_minimum_required(VERSION 3.8)
project(robot_sim)
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  robot_sim/nodes/truth_odom_node.py
  robot_sim/nodes/map_odom_static_node.py
  robot_sim/scripts/e10_demo_driver.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  robot_sim/scripts/isaac_sim_bridge.py
  robot_sim/scripts/scene_from_scenario.py
  DESTINATION share/${PROJECT_NAME}/scripts
)
ament_package()
