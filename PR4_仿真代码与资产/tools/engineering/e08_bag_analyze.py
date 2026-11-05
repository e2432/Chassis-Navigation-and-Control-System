#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E08 记录分析：从 rosbag2 里算出真实频率、重复发布、丢帧与点云几何。

要点：isaacsim.ros2.bridge 的发布节点挂在 OnPlaybackTick 上，每个渲染帧都会把
"当前值"再发一次。所以消息数 ≠ 数据更新次数。必须按 header.stamp 去重，才得到
传感器真实的更新频率；重复发布数单独统计。

用法: python3 tools/engineering/e08_bag_analyze.py --bag results/w2/e08/bag --out results/w2/e08
产出: bag_analysis.txt, cloud_frames.csv, imu_frames.csv, cloud_sample.npz
"""
import argparse
import csv
import math
import os
import statistics as st
import struct
import sys

import numpy as np


def decode_cloud(msg):
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    ox, oy, oz = fields["x"].offset, fields["y"].offset, fields["z"].offset
    step, n = msg.point_step, msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    out = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        b = i * step
        out[i, 0] = struct.unpack_from("<f", raw, b + ox)[0]
        out[i, 1] = struct.unpack_from("<f", raw, b + oy)[0]
        out[i, 2] = struct.unpack_from("<f", raw, b + oz)[0]
    return out


def stamp_key(h):
    return (h.stamp.sec, h.stamp.nanosec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="results/w2/e08/bag")
    ap.add_argument("--out", default="results/w2/e08")
    a = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap" if os.path.exists(
        os.path.join(a.bag, "bag_0.mcap")) else "sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {k: get_message(v) for k, v in types.items()}

    clouds = {}      # stamp -> np.ndarray
    cloud_order = []
    imus = {}        # stamp -> (ax,ay,az,wx,wy,wz)
    imu_order = []
    clock = []
    counts = {}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        m = deserialize_message(data, cls[topic])
        if topic == "/clock":
            clock.append((t_ns, m.clock.sec + m.clock.nanosec * 1e-9))
        elif topic == "/lidar/points":
            k = stamp_key(m.header)
            if k not in clouds:
                clouds[k] = decode_cloud(m)
                cloud_order.append(k)
        elif topic == "/imu":
            k = stamp_key(m.header)
            if k not in imus:
                la, av = m.linear_acceleration, m.angular_velocity
                imus[k] = (la.x, la.y, la.z, av.x, av.y, av.z)
                imu_order.append(k)

    lines = []
    say = lambda s: (print(s), lines.append(s))
    say("E08 记录分析（rosbag2，%s）" % a.bag)
    say("bag 原始消息数: %s" % ", ".join("%s=%d" % (k, v) for k, v in sorted(counts.items())))
    t0 = min(c[0] for c in clock) * 1e-9
    t1 = max(c[0] for c in clock) * 1e-9
    sim = [c[1] for c in clock]
    sim_span = max(sim) - min(sim)
    wall_span = t1 - t0
    say("仿真时间跨度 %.2f s / 录音墙钟 %.2f s（实时因子 %.2f）" % (sim_span, wall_span, sim_span / wall_span))
    dsteps = [b - a2 for a2, b in zip(sim[:-1], sim[1:]) if b - a2 > 0]
    say("/clock: %d 条，仿真步长中位 %.2f ms / max %.2f ms"
        % (counts.get("/clock", 0), st.median(dsteps) * 1e3, max(dsteps) * 1e3))
    say("")

    def rate_table(name, total, uniq, nominal, span):
        say("%-13s 原始 %5d 条 / 去重后 %5d 帧 → 原始 %6.2f Hz，真实更新 %6.2f Hz，"
            "重复倍数 %.2f×（标称 %.0f Hz，缺口 %.0f 帧 / %.1f%%）"
            % (name, total, uniq, total / span, uniq / span, total / max(1, uniq),
               nominal, max(0, round(nominal * span) - uniq),
               100.0 * max(0.0, 1 - uniq / max(1e-9, nominal * span))))

    rate_table("/lidar/points", counts.get("/lidar/points", 0), len(cloud_order), 10.0, sim_span)
    rate_table("/imu", counts.get("/imu", 0), len(imu_order), 200.0, sim_span)
    say("")

    # ---- 点云几何：逐帧最近点方位（雷达系）+ 距离 ----
    rows = []
    for k in cloud_order:
        P = clouds[k]
        if len(P) == 0:
            continue
        rng = np.hypot(P[:, 0], P[:, 1])
        i = int(np.argmin(rng))
        rows.append({
            "stamp_sec": k[0] + k[1] * 1e-9,
            "n_points": len(P),
            "r_min": float(rng.min()),
            "r_max": float(rng.max()),
            "z_min": float(P[:, 2].min()),
            "z_max": float(P[:, 2].max()),
            "nearest_x": float(P[i, 0]),
            "nearest_y": float(P[i, 1]),
            "nearest_z": float(P[i, 2]),
            "nearest_az_deg": math.degrees(math.atan2(P[i, 1], P[i, 0])),
        })
    if rows:
        with open(os.path.join(a.out, "cloud_frames.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            for row in rows:
                w.writerow({k2: ("%.4f" % v if isinstance(v, float) else v) for k2, v in row.items()})
        say("点云 %d 帧: 点数 %d..%d，最近距离 %.2f..%.2f m，Z 范围 %.2f..%.2f m"
            % (len(rows), min(r2["n_points"] for r2 in rows), max(r2["n_points"] for r2 in rows),
               min(r2["r_min"] for r2 in rows), max(r2["r_min"] for r2 in rows),
               min(r2["z_min"] for r2 in rows), max(r2["z_max"] for r2 in rows)))
        say("最近点方位角随时间: %s" % " ".join("%.0f°" % r2["nearest_az_deg"] for r2 in rows[::max(1, len(rows) // 12)]))

    # ---- IMU ----
    irows = []
    for k in imu_order:
        ax, ay, az, wx, wy, wz = imus[k]
        irows.append({"stamp_sec": k[0] + k[1] * 1e-9, "ax": ax, "ay": ay, "az": az,
                      "wx": wx, "wy": wy, "wz": wz})
    if irows:
        with open(os.path.join(a.out, "imu_frames.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(irows[0].keys()))
            w.writeheader()
            for row in irows:
                w.writerow({k2: ("%.5f" % v if isinstance(v, float) else v) for k2, v in row.items()})
        azs = [r2["az"] for r2 in irows]
        wws = [r2["wz"] for r2 in irows]
        say("IMU %d 帧: az 中位 %.3f（重力，静止段应≈9.81），|ax|/|ay| 中位 %.4f/%.4f"
            % (len(irows), st.median(azs), st.median([abs(r2["ax"]) for r2 in irows]),
               st.median([abs(r2["ay"]) for r2 in irows])))
        say("IMU wz: 正样本 %d，负样本 %d，|wz| max %.3f rad/s"
            % (sum(1 for v in wws if v > 0.05), sum(1 for v in wws if v < -0.05),
               max(abs(v) for v in wws)))

    if cloud_order:
        np.savez_compressed(os.path.join(a.out, "cloud_sample.npz"),
                            stamps=np.array([k[0] + k[1] * 1e-9 for k in cloud_order]),
                            clouds=np.array([clouds[k] for k in cloud_order], dtype=object),
                            allow_pickle=True)
        say("样例点云已存 cloud_sample.npz（%d 帧）" % len(cloud_order))

    with open(os.path.join(a.out, "bag_analysis.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
