from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

try:
    import rclpy
    from go2_driver.msg import LegSensor
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
except Exception:
    rclpy = None
    LegSensor = None
    Node = object
    SingleThreadedExecutor = None


LEG_NAMES = ("FL", "FR", "RL", "RR")


class RosLegSensorRecorder(Node):
    def __init__(self, topic: str, max_samples: int):
        super().__init__("contact_force_threshold_recorder")
        self.max_samples = max_samples
        self.time_history: List[float] = []
        self.force_history: Dict[str, List[float]] = {leg: [] for leg in LEG_NAMES}
        self.done = False
        self.sub = self.create_subscription(LegSensor, topic, self.on_msg, 100)

    def on_msg(self, msg: LegSensor) -> None:
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.time_history.append(stamp)
        for i, leg in enumerate(LEG_NAMES):
            self.force_history[leg].append(float(msg.foot_force[i]))
        if self.max_samples > 0 and len(self.time_history) >= self.max_samples:
            self.done = True

    def export(self) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
        t = np.asarray(self.time_history, dtype=np.float64)
        forces: Dict[str, np.ndarray] = {}
        for leg in LEG_NAMES:
            forces[leg] = np.asarray(self.force_history[leg], dtype=np.float64)
        return t, forces


def read_ros_force(
    topic: str,
    duration_sec: float,
    max_samples: int,
    spin_timeout_sec: float,
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    if rclpy is None or LegSensor is None or SingleThreadedExecutor is None:
        raise RuntimeError("当前环境不可用 rclpy 或 go2_driver Python消息接口")

    rclpy.init(args=None)
    node = RosLegSensorRecorder(topic=topic, max_samples=max_samples)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    start = time.monotonic()

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=spin_timeout_sec)
            if node.done:
                break
            if duration_sec > 0.0 and (time.monotonic() - start) >= duration_sec:
                break
    finally:
        t, forces = node.export()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

    if t.size == 0:
        raise RuntimeError("未接收到任何LegSensor数据")
    return t, forces


def to_relative_time(t: np.ndarray) -> np.ndarray:
    if t.size == 0:
        return t
    return t - t[0]


def build_visualization_payload(
    t: np.ndarray,
    forces: Dict[str, np.ndarray],
    topic: str,
    relative_time: bool,
) -> dict:
    return {
        "topic": topic,
        "relative_time": relative_time,
        "sample_count": int(t.size),
        "time": t.tolist(),
        "force": {leg: forces[leg].tolist() for leg in LEG_NAMES},
    }


def save_json(payload: dict, output: Path) -> None:
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def save_csv(t: np.ndarray, forces: Dict[str, np.ndarray], output: Path) -> None:
    rows = ["time,FL,FR,RL,RR"]
    n = int(t.size)
    for i in range(n):
        rows.append(
            f"{t[i]:.9f},{forces['FL'][i]:.9f},{forces['FR'][i]:.9f},"
            f"{forces['RL'][i]:.9f},{forces['RR'][i]:.9f}"
        )
    output.write_text("\n".join(rows), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--topic", type=str, default="/leg_sensor")
    p.add_argument("--duration-sec", type=float, default=60.0)
    p.add_argument("--max-samples", type=int, default=0)
    p.add_argument("--spin-timeout-sec", type=float, default=0.05)
    p.add_argument("--output-json", type=Path, default=Path("leg_force_vis.json"))
    p.add_argument("--output-csv", type=Path, default=Path("leg_force_vis.csv"))
    p.add_argument("--absolute-time", action="store_true")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    t, forces = read_ros_force(
        topic=args.topic,
        duration_sec=args.duration_sec,
        max_samples=args.max_samples,
        spin_timeout_sec=args.spin_timeout_sec,
    )
    if not args.absolute_time:
        t = to_relative_time(t)
    payload = build_visualization_payload(
        t=t,
        forces=forces,
        topic=args.topic,
        relative_time=not args.absolute_time,
    )
    save_json(payload, args.output_json)
    save_csv(t, forces, args.output_csv)
    print(f"采样点数量: {t.size}")
    print(f"JSON可视化数据已写入: {args.output_json}")
    print(f"CSV可视化数据已写入: {args.output_csv}")


if __name__ == "__main__":
    main()
