# 单个动态障碍状态（冻结 v1，接口契约 v1.4；不加字段）
uint32 id                          # 帧间稳定关联 id
geometry_msgs/Point position       # m，odom 帧
geometry_msgs/Vector3 velocity     # m/s，odom 帧
float32 radius                     # m，单圆近似
builtin_interfaces/Time stamp      # 该障碍的估计时刻
