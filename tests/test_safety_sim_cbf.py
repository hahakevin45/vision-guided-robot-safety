"""M4：第一個論文方法——簡化版 diff-drive CBF（control barrier function）。

驗收（工作流驗證）：從 filters/ 加一個檔案，S1–S7 全表通過，
證明「論文 → 程式 → 比較表」這條路是通的。
"""
import math

import pytest

from vgr_core.motion import DiffDriveParams
from safety_sim import metrics
from safety_sim.filters import available_filters, make_filter
from safety_sim.filters.cbf import CbfFilter
from safety_sim.runner import run_scenario
from safety_sim.scenarios import all_scenario_names, get_scenario
from vgr_core.safety import Circle, Observation, Pose, StaticInfo, Twist

FENCE = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))
STATIC = StaticInfo(params=DiffDriveParams(), robot_radius_m=0.10,
                    geofence=FENCE, max_v_mps=0.15, max_omega_rad_s=1.5)


def make_cbf():
    f = CbfFilter()
    f.reset(STATIC)
    return f


def obs(pose, obstacles=(), pose_age=0.0, link_age=0.0):
    return Observation(pose=pose, pose_age_s=pose_age,
                       wheel_feedback=(0.0, 0.0),
                       obstacles=tuple(obstacles), link_age_s=link_age)


def test_registered_in_filter_registry():
    assert "cbf" in available_filters()
    assert make_filter("cbf").name == "cbf"


def test_iccbf_registered_in_filter_registry():
    assert "iccbf" in available_filters()
    assert make_filter("iccbf").name == "iccbf"


def test_open_space_passes_command_through():
    f = make_cbf()
    d = f.filter(Twist(0.15, 0.0), obs(Pose(1.0, 0.0, 0.0)), t=0.0, dt=0.05)
    assert d.mode == "PASS"
    assert d.cmd == Twist(0.15, 0.0)


def test_facing_wall_close_speed_is_reduced():
    # 距右牆 0.35m、正對牆：CBF 須把速度壓低於全速。
    f = make_cbf()
    d = f.filter(Twist(0.15, 0.0), obs(Pose(3.65, 0.0, 0.0)), t=0.0, dt=0.05)
    assert d.mode in ("MODIFIED", "STOP")
    assert d.cmd.v < 0.15


def test_facing_away_from_wall_not_restricted():
    # 一樣近，但車頭朝反方向：遠離邊界的命令不該被擋。
    f = make_cbf()
    d = f.filter(Twist(0.15, 0.0), obs(Pose(3.65, 0.0, math.pi)), t=0.0, dt=0.05)
    assert d.cmd.v > 0.1


def test_circle_obstacle_restricts_speed():
    f = make_cbf()
    d = f.filter(Twist(0.15, 0.0),
                 obs(Pose(1.0, 0.0, 0.0), obstacles=[Circle(1.5, 0.0, 0.15)]),
                 t=0.0, dt=0.05)
    assert d.cmd.v < 0.15


def test_stale_pose_forces_stop():
    f = make_cbf()
    d = f.filter(Twist(0.15, 0.0), obs(Pose(1.0, 0.0, 0.0), pose_age=1.0),
                 t=0.0, dt=0.05)
    assert d.mode == "STOP"
    d = f.filter(Twist(0.15, 0.0), obs(None, pose_age=math.inf), t=0.0, dt=0.05)
    assert d.mode == "STOP"


def test_overspeed_still_clamped():
    f = make_cbf()
    d = f.filter(Twist(5.0, 4.0), obs(Pose(1.0, 0.0, 0.0)), t=0.0, dt=0.05)
    assert abs(d.cmd.v) <= 0.15 + 1e-9
    assert abs(d.cmd.omega) <= 1.5 + 1e-9


def test_debug_exports_min_h():
    f = make_cbf()
    d = f.filter(Twist(0.15, 0.0), obs(Pose(3.65, 0.0, 0.0)), t=0.0, dt=0.05)
    assert "min_h" in d.debug


# --- 整表：CBF 是第一個通過 S1–S7 全部情境的方法 ---

@pytest.mark.parametrize("scenario_name", ["S1", "S2", "S3", "S4", "S5", "S6", "S7"])
def test_cbf_passes_all_scenarios(scenario_name):
    scenario = get_scenario(scenario_name)
    trace = run_scenario(scenario, make_filter("cbf"))
    passed, reasons = scenario.evaluate(trace)
    assert passed, (scenario_name, reasons, metrics.summarize(trace, scenario.fault_t0))
