cmake_minimum_required(VERSION 3.8)
project(robot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# E09：TF 树与真值里程计的 ROS 2 节点（纯 Python，按脚本安装，保持 ament_cmake 构建方式）
install(PROGRAMS
  robot_sim/nodes/truth_odom_node.py
  robot_sim/nodes/map_odom_static_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# E09/E10：Isaac Sim 侧统一桥接脚本（由 launch 用 conda Python 调起，按文件安装到 share）
install(FILES robot_sim/scripts/isaac_sim_bridge.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(PROGRAMS
  robot_sim/scripts/e10_demo_driver.py
  robot_sim/scripts/e10_motion_probe.py
  robot_sim/scripts/scene_from_scenario.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
