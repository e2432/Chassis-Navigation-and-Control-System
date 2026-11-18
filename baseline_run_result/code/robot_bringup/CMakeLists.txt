cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# 行人横穿走廊注入层（9-3）：Nav2 static_layer 的 /map_updates 通道实测无效，
# 改由本层自己注入 + 自己擦除（见 src/ped_corridor_layer.cpp 顶部说明）
add_library(ped_corridor_layer SHARED src/ped_corridor_layer.cpp)
ament_target_dependencies(ped_corridor_layer rclcpp nav2_costmap_2d nav_msgs pluginlib)
install(TARGETS ped_corridor_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

# 启动文件 + 共享配置（三基线参数、costmap/planner、平面限速参数）
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 共享执行层平面限速节点（9-1 §2：三基线 + MPC 同等生效）
install(PROGRAMS
  scripts/flat_speed_limiter.py
  scripts/safety_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
