"""Active-field three-arm experiment: trace recorder evidence surface.

Task 5 的指定序列化測試：raw/gated ArUco pose、odom、以及 marker-ID
觀察都必須以各自 topic 寫入同一 JSONL，並保持既有 topic/shape 相容。
"""
import json

from gazebo_sim.nodes.trace_recorder import TraceRecorderCore
from vgr_core.safety import Pose, Twist


def test_recorder_distinguishes_raw_and_gated_pose_and_records_odom_ids():
    core = TraceRecorderCore()
    core.record_aruco_pose(
        1.0, Pose(1.0, 2.0, 0.3), stamp_s=0.9, topic="/aruco/pose_raw")
    core.record_aruco_pose(
        1.1, Pose(1.0, 2.0, 0.3), stamp_s=0.9, topic="/aruco/pose")
    core.record_odom(1.2, Pose(1.1, 2.0, 0.3), Twist(0.1, 0.0))
    core.record_marker_ids(1.3, stamp_s=1.25, ids=(0, 5))
    rows = [json.loads(line) for line in core.to_jsonl().splitlines()]
    assert [row["topic"] for row in rows] == [
        "/aruco/pose_raw", "/aruco/pose", "/odom", "/aruco/marker_ids"]
    assert rows[2]["pose"]["x"] == 1.1
    assert rows[2]["twist"]["v"] == 0.1
    assert rows[3] == {
        "topic": "/aruco/marker_ids", "t": 1.3,
        "stamp_s": 1.25, "ids": [0, 5],
    }
