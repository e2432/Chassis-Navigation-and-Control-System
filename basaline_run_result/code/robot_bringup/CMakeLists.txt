cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

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
