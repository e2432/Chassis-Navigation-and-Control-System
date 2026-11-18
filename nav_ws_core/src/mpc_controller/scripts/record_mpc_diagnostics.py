#!/usr/bin/env python3
"""Zero-math ROS adapter: record /mpc/diagnostics to CSV.

The numerical aggregation (qp_timing mean/p95/max) is done offline in the
conda `pro` environment by prototype/mpc_numpy/aggregate_qp_timing.py.
"""

from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node

from mpc_controller.msg import MpcDiagnostics


FIELDS = [
    "stamp_sec", "stamp_nanosec", "solve_ms", "qp_iterations", "status",
    "fallback_streak", "alarm", "obstacle_count", "static_field_used",
    "success", "fallback", "max_slack",
]


class DiagnosticsRecorder(Node):
    def __init__(self):
        super().__init__("mpc_diagnostics_recorder")
        self.declare_parameter("output_file", "/tmp/mpc_diagnostics.csv")
        self.declare_parameter("topic", "/mpc/diagnostics")
        output = Path(str(self.get_parameter("output_file").value))
        topic = str(self.get_parameter("topic").value)
        output.parent.mkdir(parents=True, exist_ok=True)

        self._stream = output.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDS)
        self._writer.writeheader()
        self._stream.flush()

        self.create_subscription(MpcDiagnostics, topic, self.callback, 50)
        self.get_logger().info(f"recording {topic} -> {output}")

    def callback(self, msg: MpcDiagnostics):
        self._writer.writerow({
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
            "solve_ms": msg.solve_ms,
            "qp_iterations": msg.qp_iterations,
            "status": msg.status,
            "fallback_streak": msg.fallback_streak,
            "alarm": int(msg.alarm),
            "obstacle_count": msg.obstacle_count,
            "static_field_used": int(msg.static_field_used),
            "success": int(msg.success),
            "fallback": int(msg.fallback),
            "max_slack": msg.max_slack,
        })
        self._stream.flush()

    def destroy_node(self):
        try:
            self._stream.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = DiagnosticsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
