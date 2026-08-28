#!/usr/bin/env python3
"""從實車 rosbag（mcap）抽軌跡成 JSON，供 Gazebo 回播與畫圖。

需要 pip 套件：mcap、mcap-ros2-support（不需要與 bag 同版的 ROS）。

用法：
    python3 tools/extract_bag_traj.py outputs/media_20260717/20260717_091715_bag \
        -o outputs/media_20260717/traj_20260717_091715.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from mcap_ros2.reader import read_ros2_messages


def yaw_of(q) -> float:
    return 2 * math.atan2(q.z, q.w)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="bag 目錄或 .mcap 檔")
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()

    p = Path(args.bag)
    mcap_path = p if p.suffix == ".mcap" else next(p.glob("*.mcap"))

    out: dict[str, list] = {"fused": [], "aruco": [], "odom": [], "status": []}
    for m in read_ros2_messages(str(mcap_path)):
        t = m.log_time_ns / 1e9
        r = m.ros_msg
        topic = m.channel.topic
        if topic == "/pose_fused":
            pp = r.pose.pose
            out["fused"].append([t, pp.position.x, pp.position.y, yaw_of(pp.orientation)])
        elif topic == "/aruco/pose":
            out["aruco"].append([t, r.pose.position.x, r.pose.position.y,
                                 yaw_of(r.pose.orientation)])
        elif topic == "/odom":
            pp = r.pose.pose
            out["odom"].append([t, pp.position.x, pp.position.y, yaw_of(pp.orientation)])
        elif topic == "/safety_gate/status":
            out["status"].append([t, json.loads(r.data)["mode"]])

    Path(args.out).write_text(json.dumps(out))
    n = {k: len(v) for k, v in out.items()}
    print(f"wrote {args.out} {n}")


if __name__ == "__main__":
    main()
