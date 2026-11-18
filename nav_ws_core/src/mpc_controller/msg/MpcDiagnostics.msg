# W6 MPC 逐拍诊断（建议新增接口，需契约变更 PR 双方确认）
std_msgs/Header header
bool success                    # 本拍求解是否成功
bool fallback                   # 本拍是否零速回退
bool alarm                      # 连续失败告警（fallback_streak >= 5）
float64 solve_ms                # MpcCore::solve 外环墙钟耗时（含 SQP）
uint32 qp_iterations            # 最后一次 OSQP 迭代次数
uint32 fallback_streak          # 连续失败拍数
float64 max_slack               # 当前拍最大 slack
uint32 obstacle_count           # /mpc/obstacles 动态障碍数
bool static_field_used          # 本拍是否使用静态距离场
string status                   # OSQP 状态字符串
