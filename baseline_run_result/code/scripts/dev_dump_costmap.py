#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把局部代价地图整张抓下来（走 nav2_msgs/srv/GetCostmap），打印 ASCII。

用法（栈已启动）：
    python3 scripts/dev_dump_costmap.py [输出文件]
"""
import sys
import time

import rclpy
from nav2_msgs.srv import GetCostmap

OUT = sys.argv[1] if len(sys.argv) > 1 else ""

rclpy.init()
node = rclpy.create_node("dev_dump_costmap")

cands = [n for (n, t) in node.get_service_names_and_types()
         if "get_costmap" in n and "local" in n]
print("候选服务:", cands, flush=True)
if not cands:
    print("未发现 get_costmap 服务", flush=True)
    sys.exit(1)
cli = node.create_client(GetCostmap, cands[0])
if not cli.wait_for_service(timeout_sec=5.0):
    print("服务不可用", flush=True)
    sys.exit(1)
req = GetCostmap.Request()
req.specs.layer = "local_costmap"
req.specs.resolution = 0.05
req.specs.size_x = 120
req.specs.size_y = 120
req.specs.origin.orientation.w = 1.0
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
res = fut.result()
if res is None:
    print("调用超时", flush=True)
    sys.exit(1)
m = res.map
md = m.metadata
print("栅格 %dx%d res=%.3f 原点(%.2f,%.2f) 层=%s" % (
    md.size_x, md.size_y, md.resolution, md.origin.position.x, md.origin.position.y,
    md.layer), flush=True)
data = m.data
if OUT:
    with open(OUT, "wb") as f:
        f.write(bytes(data))
    print("已写出", OUT, flush=True)

w, h = md.size_x, md.size_y
step = max(1, w // 48)
lut = {0: ".", 99: "i", 100: "#", 255: "?"}
counts = {}
for v in data:
    counts[v] = counts.get(v, 0) + 1
top = sorted(counts.items(), key=lambda kv: -kv[1])[:8]
print("取值分布(前8):", top, flush=True)
for jy in range(h - 1, -1, -step):
    row = ""
    for jx in range(0, w, step):
        v = data[jy * w + jx]
        row += lut.get(v, " " if v <= 98 else ("i" if v <= 252 else "#"))
    print("y=%+6.2f %s" % (md.origin.position.y + (jy + 0.5) * md.resolution, row), flush=True)
rclpy.shutdown()
