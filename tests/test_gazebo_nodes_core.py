import json
import math
from pathlib import Path

import pytest

from gazebo_sim.nodes.pseudo_aruco import PseudoArucoCore
from gazebo_sim.nodes.safety_gate import SafetyGateCore
from gazebo_sim.nodes.scripted_nav import ScriptedNavCore
from gazebo_sim.nodes.trace_recorder import TraceRecorderCore
from safety_sim.filters import make_filter
from vgr_core.safety import Circle, Pose, SafetyDecision, Twist


class RecordingFilter:
    name = "recording"

    def __init__(self) -> None:
        self.static_info = None
        self.calls = []

    def reset(self, static_info) -> None:
        self.static_info = static_info

    def filter(self, desired, obs, t, dt):
        self.calls.append((desired, obs, t, dt))
        return SafetyDecision(cmd=Twist(desired.v / 2.0, desired.omega), mode="MODIFIED",
                              debug={"pose_age_s": obs.pose_age_s})


def test_pseudo_aruco_core_reuses_safety_sim_localizer_and_freezes_stamp_during_dropout():
    core = PseudoArucoCore(update_hz=10.0, noise_xy_std=0.0, noise_theta_std=0.0, seed=0)
    core.set_dropout(False)
    core.update_true_pose(Pose(1.0, 2.0, 0.25))

    first = core.tick(0.0)
    assert first is not None
    assert first.pose == Pose(1.0, 2.0, 0.25)
    assert first.stamp_s == pytest.approx(0.0)

    core.set_dropout(True)
    core.update_true_pose(Pose(3.0, 4.0, 1.0))
    frozen = core.tick(0.5)

    assert frozen is not None
    assert frozen.pose == Pose(1.0, 2.0, 0.25)
    assert frozen.stamp_s == pytest.approx(0.0)
    assert frozen.age_s == pytest.approx(0.5)


def test_safety_gate_core_builds_observation_from_message_stamp_and_calls_filter():
    filt = RecordingFilter()
    core = SafetyGateCore(filt, max_v_mps=0.15, max_omega_rad_s=1.5)
    core.update_nav(Twist(0.4, 0.2), stamp_s=9.8)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=9.25)

    out = core.tick(now_s=10.0)

    assert out.cmd == Twist(0.2, 0.2)
    assert out.mode == "MODIFIED"
    desired, obs, t, dt = filt.calls[-1]
    assert desired == Twist(0.4, 0.2)
    assert obs.pose == Pose(1.0, 0.0, 0.0)
    assert obs.pose_age_s == pytest.approx(0.75)
    assert obs.link_age_s == 0.0
    assert t == pytest.approx(10.0)
    assert dt == pytest.approx(0.05)
    assert filt.static_info.max_v_mps == pytest.approx(0.15)


def test_safety_gate_core_uses_infinite_age_before_first_pose():
    filt = RecordingFilter()
    core = SafetyGateCore(filt, max_v_mps=0.15, max_omega_rad_s=1.5)

    core.tick(now_s=1.0)

    obs = filt.calls[-1][1]
    assert obs.pose is None
    assert math.isinf(obs.pose_age_s)


def test_safety_gate_core_wheel_feedback():
    filt = RecordingFilter()
    core = SafetyGateCore(filt)

    # 沒呼叫時預設 (0,0)
    obs_default, _ = core.build_observation(now_s=1.0)
    assert obs_default.wheel_feedback == (0.0, 0.0)

    # 呼叫 update_wheel_feedback 後 build_observation 帶出該值
    core.update_wheel_feedback(0.12, -0.05)
    obs_updated, _ = core.build_observation(now_s=1.0)
    assert obs_updated.wheel_feedback == (0.12, -0.05)


def test_safety_gate_stops_when_nav_command_is_stale():
    filt = RecordingFilter()
    core = SafetyGateCore(filt, nav_timeout_s=0.5)
    core.update_nav(Twist(0.1, 0.0), stamp_s=1.0)

    fresh = core.tick(1.4)
    stale = core.tick(1.6)

    assert fresh.cmd.v > 0.0
    assert stale.cmd == Twist.stop()
    assert filt.calls[-1][0] == Twist.stop()


def test_ros_wrapper_defaults_to_safe_apf():
    text = Path("gazebo_sim/nodes/safety_gate.py").read_text(encoding="utf-8")
    assert 'declare_parameter("filter_name", "safe_apf")' in text
    assert 'declare_parameter("nav_timeout_s", 0.2)' in text


def test_gate_declares_ignore_pose_drift_param():
    """R3 漂移實驗臂：wrapper 須宣告並透傳 ignore_pose_drift 給 SAPF filter。

    Regression：filter kwargs 若只吃 filter_name=cbf 的 cbf_* 參數，SAPF 漂移
    實驗臂就無法關閉 pose drift 抑制。
    """
    text = Path("gazebo_sim/nodes/safety_gate.py").read_text(encoding="utf-8")
    assert 'declare_parameter("filter_kwargs_ignore_pose_drift", False)' in text
    assert 'filter_kwargs["ignore_pose_drift"] = True' in text


def test_gate_declares_fixed_d_safe_param():
    """R3 固定安全半徑實驗臂：wrapper 須宣告 fixed_d_safe_m 並透傳給 SAPF filter。

    Reset 後覆寫 d_safe/d_vort/Q* 的實驗參數，只能經 filter kwargs 進入 filter。
    """
    text = Path("gazebo_sim/nodes/safety_gate.py").read_text(encoding="utf-8")
    assert 'declare_parameter("filter_kwargs_fixed_d_safe_m", -1.0)' in text
    assert 'filter_kwargs["fixed_d_safe_m"]' in text


def test_gate_core_dynamic_obstacle_update():
    """資訊對等：core 可動態更新 obstacles（視覺量測與 DWB/RPP 同源）。

    /obstacles_measured 的 String JSON 解析後經 update_obstacles 進入
    Observation.obstacles；此測試驗證 core 層的更新介面。
    """
    core = SafetyGateCore(
        make_filter("safe_apf_new"), max_v_mps=0.15, max_omega_rad_s=1.5)
    core.update_obstacles((Circle(2.0, 0.0, 0.2),))
    assert core._obstacles == (Circle(2.0, 0.0, 0.2),)


def test_scripted_nav_core_maps_gs_profiles_to_safety_sim_nav_definitions():
    gs1 = ScriptedNavCore("gs1_wall_rush")
    gs2 = ScriptedNavCore("gs2_blackout")

    assert gs1.command(now_s=0.0) == Twist(2.0, 0.0)
    assert gs2.command(now_s=10.0) == Twist(0.15, 0.0)


def test_trace_recorder_core_writes_topic_events_as_jsonl():
    recorder = TraceRecorderCore()
    recorder.record_true_pose(1.0, Pose(0.5, 0.0, 0.0), Twist(0.23, 0.04))
    recorder.record_twist("/cmd_vel_nav", 1.0, Twist(0.2, 0.0))
    recorder.record_twist("/cmd_vel_safe", 1.05, Twist(0.1, 0.0))
    recorder.record_aruco_pose(1.0, Pose(0.5, 0.0, 0.0), stamp_s=0.95)
    recorder.record_status(1.05, "MODIFIED", {"k": 2.0})

    rows = [json.loads(line) for line in recorder.to_jsonl().splitlines()]

    assert rows[0]["topic"] == "/sim/true_pose"
    assert rows[0]["true_pose"] == {"x": 0.5, "y": 0.0, "theta": 0.0}
    assert rows[0]["actual_twist"] == {"v": 0.23, "omega": 0.04}
    assert rows[2]["topic"] == "/cmd_vel_safe"
    assert rows[2]["twist"] == {"v": 0.1, "omega": 0.0}
    assert rows[3]["stamp_s"] == pytest.approx(0.95)
    assert rows[4]["mode"] == "MODIFIED"


def test_pseudo_aruco_wrapper_exposes_dropout_service():
    """GS2 故障注入靠 /aruco/set_dropout service；wrapper 必須建立它。

    Regression：clean-cutover 遷移曾丟失 service 建立，導致 GS2 腳本的
    `ros2 service call /aruco/set_dropout` 永遠找不到 service。
    """
    source = Path(__file__).parent.parent.joinpath(
        "gazebo_sim", "nodes", "pseudo_aruco.py"
    ).read_text(encoding="utf-8")
    assert 'SetBool, "/aruco/set_dropout"' in source
    assert "self._core.set_dropout(bool(request.data))" in source


def test_pseudo_aruco_core_positional_dropout_after_x():
    """位置型切斷：車走過 dropout_after_x 後 /aruco/pose 停更（age 增長）。

    模擬實車走出 marker 覆蓋區——比時間型 dropout 更接近「從某個 x 之後
    就看不到」的語意。
    """
    core = PseudoArucoCore(update_hz=10.0, noise_xy_std=0.0,
                           noise_theta_std=0.0, seed=0,
                           dropout_after_x=1.5)
    core.update_true_pose(Pose(1.0, 0.0, 0.0))
    before = core.tick(0.0)
    assert before is not None  # x=1.0 < 1.5：正常發布

    core.update_true_pose(Pose(1.6, 0.0, 0.0))
    after = core.tick(0.2)
    assert after is not None
    assert after.pose == Pose(1.0, 0.0, 0.0)  # 凍結最後一筆
    assert after.age_s == pytest.approx(0.2)  # age 增長（gate 據此 dead-reckoning）

    core.update_true_pose(Pose(2.5, 0.0, 0.0))
    later = core.tick(1.0)
    assert later.pose == Pose(1.0, 0.0, 0.0)
    assert later.age_s == pytest.approx(1.0)


def test_pseudo_aruco_core_positional_window_events():
    core = PseudoArucoCore(update_hz=10.0, noise_xy_std=0.0,
                           noise_theta_std=0.0, seed=0,
                           dropout_after_x=1.5)
    core.update_true_pose(Pose(1.0, 0.0, 0.0))
    core.tick(0.0)
    assert core.pop_window_events() == []   # 未觸發

    core.update_true_pose(Pose(1.7, 0.0, 0.0))
    core.tick(2.0)
    events = core.pop_window_events()
    assert len(events) == 1
    t0, t1 = events[0]
    assert t0 == pytest.approx(2.0)
    assert t1 is None                       # 位置型：視窗開到 run 結束


def test_pseudo_aruco_core_no_positional_dropout_by_default():
    core = PseudoArucoCore(update_hz=10.0, noise_xy_std=0.0,
                           noise_theta_std=0.0, seed=0)
    core.update_true_pose(Pose(5.0, 0.0, 0.0))
    assert core.tick(0.0) is not None


def test_trace_recorder_core_records_window_event():
    from gazebo_sim.nodes.trace_recorder import TraceRecorderCore

    core = TraceRecorderCore()
    core.record_event(12.5, "/aruco/dropout_window", {"t0": 12.5, "t1": None})
    assert core.to_jsonl() == (
        '{"t": 12.5, "t0": 12.5, "t1": null, "topic": "/aruco/dropout_window"}')


def test_pseudo_aruco_core_resume_after_x():
    """盲走後接回視覺：車過 resume_after_x 後恢復發布新鮮 pose。

    對應實車「走出 marker 覆蓋區又看到下一個 marker」——定位校正、
    誤差歸零、機器繼續跑。
    """
    core = PseudoArucoCore(update_hz=10.0, noise_xy_std=0.0,
                           noise_theta_std=0.0, seed=0,
                           dropout_after_x=1.2, resume_after_x=2.5)
    # 新鮮段（建立首筆 fix）
    core.update_true_pose(Pose(1.0, 0.0, 0.0))
    fresh = core.tick(0.0)
    assert fresh is not None
    assert fresh.age_s == pytest.approx(0.0)
    core.update_true_pose(Pose(1.1, 0.0, 0.0))
    core.tick(0.5)
    # 斷視覺（x > 1.2）
    core.update_true_pose(Pose(1.5, 0.0, 0.0))
    blind = core.tick(1.0)
    assert blind.pose == Pose(1.1, 0.0, 0.0)   # 凍結（最後 fix）
    assert blind.age_s == pytest.approx(0.5)   # age = t − 最後 fix (1.0−0.5)
    # 盲走中（x 在 1.2-2.5 之間）
    core.update_true_pose(Pose(2.0, 0.0, 0.0))
    still_blind = core.tick(2.0)
    assert still_blind.age_s == pytest.approx(1.5)   # 2.0−0.5
    # 接回（x > 2.5）：恢復新鮮
    core.update_true_pose(Pose(2.6, 0.0, 0.0))
    resumed = core.tick(3.0)
    assert resumed is not None
    assert resumed.pose == Pose(2.6, 0.0, 0.0)  # 新鮮（非凍結）
    assert resumed.age_s == pytest.approx(0.0)
    # 視窗事件：t1 = 恢復時刻
    events = core.pop_window_events()
    assert len(events) == 1
    t0, t1 = events[0]
    assert t0 == pytest.approx(1.0)
    assert t1 == pytest.approx(3.0)


def test_pseudo_aruco_core_resume_optional_keeps_permanent():
    """resume_after_x=None（預設）：永久切斷（t1=null，原行為）。"""
    core = PseudoArucoCore(update_hz=10.0, noise_xy_std=0.0,
                           noise_theta_std=0.0, seed=0,
                           dropout_after_x=1.2)
    core.update_true_pose(Pose(1.0, 0.0, 0.0))
    core.tick(0.0)   # 建立 fix
    core.update_true_pose(Pose(1.5, 0.0, 0.0))
    core.tick(1.0)
    core.update_true_pose(Pose(3.0, 0.0, 0.0))
    still = core.tick(3.0)
    assert still.age_s == pytest.approx(3.0)   # 仍凍結
    t0, t1 = core.pop_window_events()[0]
    assert t1 is None


def test_pseudo_aruco_core_resume_does_not_retrigger_dropout():
    """resume 後車已過 dropout 線：不得再次觸發 dropout（窗口事件風暴 bug）。"""
    core = PseudoArucoCore(update_hz=10.0, noise_xy_std=0.0,
                           noise_theta_std=0.0, seed=0,
                           dropout_after_x=1.2, resume_after_x=2.5)
    core.update_true_pose(Pose(1.0, 0.0, 0.0))
    core.tick(0.0)
    core.update_true_pose(Pose(1.3, 0.0, 0.0))
    core.tick(1.0)      # 觸發 dropout
    core.update_true_pose(Pose(2.6, 0.0, 0.0))
    resumed = core.tick(2.0)   # resume（fresh）
    assert resumed is not None and resumed.age_s == pytest.approx(0.0)
    # 車繼續前進（x 遠超 dropout 線）：不得重新 dropout
    core.update_true_pose(Pose(2.9, 0.0, 0.0))
    after = core.tick(2.5)
    assert after.age_s == pytest.approx(0.0)   # 仍新鮮
    events = core.pop_window_events()
    assert len(events) == 1   # 只有一對 (t0, t1)
    assert events[0][1] == pytest.approx(2.0)
