#!/usr/bin/env python3
"""W3 NavFn 取源 CLI（系统 Python 3.10 + rclpy；被 run_w3.py 以 subprocess 调用）。

子命令（docs/MPC公式化.md §12：一次性取路径，之后全 numpy 推演）：
  load-map --yaml <abs>             调 map_server LoadMap 切换地图
  get-path --start x y yaw --goal x y yaw
                                    一次 ComputePathToPose，输出折线
约定：stdout 只出一行 JSON；诊断日志一律 stderr；成功 exit 0 / 失败 exit 2。
注意：本文件禁止被 conda 侧 import（那边无 rclpy），只能以 /usr/bin/python3 执行。
"""
from __future__ import annotations

import argparse
import json
import math
import sys


def _out(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False))


def _fail(reason: str) -> int:
    _out({"status": "error", "reason": reason})
    return 2


def _pose_stamped(pose_cls, x: float, y: float, yaw: float, frame_id: str = "map"):
    p = pose_cls()
    p.header.frame_id = frame_id
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    # 平面 yaw → 四元数（不引 tf）
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


def cmd_load_map(args) -> int:
    import rclpy
    from nav2_msgs.srv import LoadMap

    rclpy.init()
    node = rclpy.create_node("w3_load_map")
    try:
        client = node.create_client(LoadMap, args.service)
        if not client.wait_for_service(timeout_sec=args.timeout):
            return _fail(f"服务 {args.service} 不可用")
        req = LoadMap.Request()
        req.map_url = args.yaml
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        if not fut.done():
            return _fail("LoadMap 调用超时")
        resp = fut.result()
        if resp is None:
            return _fail("LoadMap 无响应")
        result = getattr(resp, "result", getattr(resp, "error", 0))
        if result != 0:
            return _fail(f"LoadMap 返回 result={result}（查 map YAML 路径/合法性）")
        _out({"status": "ok"})
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cmd_get_path(args) -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node("w3_get_path")
    try:
        client = ActionClient(node, ComputePathToPose, args.action)
        if not client.wait_for_server(timeout_sec=args.timeout):
            return _fail(f"action {args.action} 不可用")
        goal = ComputePathToPose.Goal()
        goal.goal = _pose_stamped(PoseStamped, *args.goal)
        goal.planner_id = ""
        if hasattr(goal, "use_start"):  # 无此字段则靠静态 TF（起点=原点）
            goal.use_start = True
            goal.start = _pose_stamped(PoseStamped, *args.start)
        send_fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_fut, timeout_sec=args.timeout)
        handle = send_fut.result() if send_fut.done() else None
        if handle is None or not handle.accepted:
            return _fail("规划目标被拒绝或发送超时（查起点/目标是否落在占据栅格）")
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=args.timeout)
        if not res_fut.done():
            return _fail("取规划结果超时")
        path = res_fut.result().result.path
        wps = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
        if len(wps) < 2:
            return _fail(f"路径点数不足: {len(wps)}")
        if any(not (math.isfinite(x) and math.isfinite(y)) for x, y in wps):
            return _fail("路径含非有限值")
        length = sum(
            math.hypot(wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1])
            for i in range(len(wps) - 1)
        )
        _out({"status": "ok", "waypoints": wps, "length": length})
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    ap = argparse.ArgumentParser(description="W3 NavFn 取源（stdout 仅一行 JSON）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_map = sub.add_parser("load-map")
    p_map.add_argument("--yaml", required=True, help="map YAML 绝对路径")
    p_map.add_argument("--service", default="/map_server/load_map")
    p_map.add_argument("--timeout", type=float, default=10.0)
    p_map.set_defaults(func=cmd_load_map)

    p_path = sub.add_parser("get-path")
    p_path.add_argument("--start", type=float, nargs=3, metavar=("X", "Y", "YAW"), required=True)
    p_path.add_argument("--goal", type=float, nargs=3, metavar=("X", "Y", "YAW"), required=True)
    p_path.add_argument("--action", default="/compute_path_to_pose")
    p_path.add_argument("--timeout", type=float, default=30.0)
    p_path.set_defaults(func=cmd_get_path)

    args = ap.parse_args()
    try:
        return args.func(args)
    except Exception as e:  # noqa: BLE001 — CLI 边界统一出 JSON
        return _fail(f"{type(e).__name__}: {e}")


if __name__ == "__main__":
    sys.exit(main())
