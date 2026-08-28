import json

import numpy as np

from gazebo_sim.nodes.aruco_detector import process_frame
from vgr_core.safety import Pose


class FakeLocalizer:
    def __init__(self, pose, ids):
        self.pose = pose
        self.ids = ids
        self.last_used_ids = []

    def locate(self, _image):
        self.last_used_ids = list(self.ids)
        return self.pose


def test_process_frame_returns_pose_and_sorted_accepted_ids():
    result = process_frame(
        FakeLocalizer(Pose(1.0, 2.0, 0.3), (5, 0)),
        np.zeros((2, 2, 3), dtype=np.uint8), stamp_s=12.25)
    assert result.pose == Pose(1.0, 2.0, 0.3)
    assert json.loads(result.marker_ids_json) == {"stamp_s": 12.25, "ids": [0, 5]}


def test_process_frame_emits_empty_id_observation_without_pose():
    result = process_frame(
        FakeLocalizer(None, ()), np.zeros((2, 2, 3), dtype=np.uint8), stamp_s=3.0)
    assert result.pose is None
    assert json.loads(result.marker_ids_json) == {"stamp_s": 3.0, "ids": []}
