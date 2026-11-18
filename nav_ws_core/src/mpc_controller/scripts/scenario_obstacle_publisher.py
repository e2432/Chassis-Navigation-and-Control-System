#!/usr/bin/env python3
"""TEST ONLY: publish scenario obstacles on /mpc/obstacles.

Reads docs/scenarios/instances.yaml (read-only) and republishes static circle
obstacles and contract pedestrians in the odom frame. Walls are NOT
representable by ObstacleState and are not published.

This node exists only to exercise the plugin's non-empty obstacle path in
smoke/integration tests. It must NOT be used for formal navigation or
benchmark results (a real tracker must publish /mpc/obstacles instead).
"""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node

from mpc_controller.msg import ObstacleState, ObstacleStateArray


def default_instances_file() -> str:
    env = os.environ.get("NAV_WS")
    if env:
        candidate = Path(env).parent / "docs" / "scenarios" / "instances.yaml"
        if candidate.exists():
            return str(candidate)
    # scripts/ -> mpc_controller/ -> src/ -> nav_ws_core/ -> repo root
    repo = Path(__file__).resolve().parents[4]
    return str(repo / "docs" / "scenarios" / "instances.yaml")


class ScenarioObstaclePublisher(Node):
    def __init__(self):
        super().__init__("scenario_obstacle_publisher")
        self.declare_parameter("instances_file", "")
        self.declare_parameter("scenario", "dynamic_pedestrian")
        self.declare_parameter("run", 1)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("time_offset", 0.0)

        parameter_file = str(self.get_parameter("instances_file").value)
        instances_file = parameter_file or default_instances_file()
        scenario = str(self.get_parameter("scenario").value)
        run = int(self.get_parameter("run").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.time_offset = float(self.get_parameter("time_offset").value)

        with open(instances_file, encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        instance = None
        for candidate in document["instances"]:
            if candidate["scenario"] == scenario and int(candidate["run"]) == run:
                instance = candidate
                break
        if instance is None:
            raise RuntimeError(
                f"scenario instance not found: {scenario} run={run} in {instances_file}")
        if document.get("generator_version") != "2.0.0":
            raise RuntimeError(
                f"unsupported instances generator_version={document.get('generator_version')}")

        self.static_obstacles = list(instance.get("obstacles", []))
        self.pedestrians = list(instance.get("pedestrians", []))
        self.walls = list(instance.get("walls", []))
        self.start_time = self.get_clock().now()

        self.publisher = self.create_publisher(ObstacleStateArray, "/mpc/obstacles", 10)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self.publish)

        self.get_logger().warn(
            "TEST ONLY: scenario=%s run=%d obstacles=%d pedestrians=%d walls=%d "
            "(walls are not published); do not use for formal runs" %
            (scenario, run, len(self.static_obstacles), len(self.pedestrians),
             len(self.walls)))

    def publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        t = (now - self.start_time).nanoseconds * 1e-9 + self.time_offset

        array = ObstacleStateArray()
        array.header.frame_id = self.frame_id
        array.header.stamp = stamp

        for index, obstacle in enumerate(self.static_obstacles):
            state = ObstacleState()
            state.id = index + 1
            state.position.x = float(obstacle["x"])
            state.position.y = float(obstacle["y"])
            state.velocity.x = 0.0
            state.velocity.y = 0.0
            state.radius = float(obstacle["r"])
            state.stamp = stamp
            array.obstacles.append(state)

        for index, pedestrian in enumerate(self.pedestrians):
            state = ObstacleState()
            state.id = 1001 + index
            state.position.x = float(pedestrian["x_cross"])
            state.position.y = float(pedestrian["dir"]) * float(pedestrian["speed"]) * (
                t - float(pedestrian["t_meet"]))
            state.velocity.x = 0.0
            state.velocity.y = float(pedestrian["dir"]) * float(pedestrian["speed"])
            state.radius = float(pedestrian["r"])
            state.stamp = stamp
            array.obstacles.append(state)

        self.publisher.publish(array)


def main():
    rclpy.init()
    node = ScenarioObstaclePublisher()
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
