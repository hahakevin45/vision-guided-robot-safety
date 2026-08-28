"""`run_active_aruco_field.sh` 的收斂版 runner 契約測試。

驗證最小化 runner（三個 arm、各一次 repeat）：
- DRY_RUN 單一 arm 寫出一份鎖定 manifest；
- `--all` 依序寫出 controlled_adaptive / controlled_fixed_028 /
  natural_adaptive 三份 repeat=1 的 manifest；
- 未知 arm 以 exit code 2 拒絕；
- 腳本文字只使用 active world 與真實 vision pipeline，絕不引用
  vgr_field_replay.world 或 pseudo_aruco。

這些都是行為/契約測試，不需要 ROS 或 Gazebo。
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPO_ROOT / "gazebo_sim" / "scripts" / "run_active_aruco_field.sh"

ARM_ORDER = ("controlled_adaptive", "controlled_fixed_028", "natural_adaptive")


def _run(*args: str, tmp_path: Path) -> subprocess.CompletedProcess:
    env = dict(os.environ, DRY_RUN="YES")
    return subprocess.run(
        ["bash", str(RUNNER), *args],
        cwd=str(REPO_ROOT),
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )


def _read_script() -> str:
    assert RUNNER.exists(), f"missing runner script: {RUNNER}"
    return RUNNER.read_text(encoding="utf-8")


def test_dry_run_single_arm_writes_locked_manifest(tmp_path):
    result = _run("--arm", "controlled_adaptive", "--repeat", "1",
                  "--out", str(tmp_path), tmp_path=tmp_path)
    assert result.returncode == 0, result.stderr
    manifests = list(tmp_path.rglob("manifest.json"))
    assert len(manifests) == 1
    manifest = json.loads(manifests[0].read_text())
    assert manifest["arm"] == "controlled_adaptive"
    assert manifest["repeat"] == 1
    assert manifest["left_wheel_mu"] == 0.03
    assert manifest["timeout_sim_s"] == 90.0


def test_all_dry_run_writes_three_manifests_in_order(tmp_path):
    result = _run("--all", "--out", str(tmp_path), tmp_path=tmp_path)
    assert result.returncode == 0, result.stderr
    manifests = [
        json.loads(p.read_text())
        for p in sorted(tmp_path.rglob("manifest.json"))
    ]
    assert len(manifests) == len(ARM_ORDER)
    assert [m["arm"] for m in manifests] == list(ARM_ORDER)
    assert {m["arm"] for m in manifests} == set(ARM_ORDER)
    assert all(m["repeat"] == 1 for m in manifests)


def test_runner_rejects_unknown_arm_with_exit_2(tmp_path):
    result = _run("--arm", "bogus", "--repeat", "1",
                  "--out", str(tmp_path), tmp_path=tmp_path)
    assert result.returncode == 2


def test_runner_script_uses_active_vision_stack_and_locked_settings():
    text = _read_script()
    for required in (
        "vgr_field_active.world",
        "gazebo_sim.nodes.aruco_detector",
        "vision_gate",
        "trace_recorder",
        "safety_gate",
        "field_dropout_controller",
        "sapf_nominal",
        "left_wheel_mu",
        "0.03",
        "90",
        "/aruco/pose_raw",
        "/aruco/pose",
    ):
        assert required in text, f"runner script missing: {required!r}"


def test_runner_script_never_references_replay_world_or_pseudo_aruco():
    text = _read_script()
    assert "vgr_field_replay.world" not in text
    assert "pseudo_aruco" not in text


def test_all_returns_nonzero_when_any_arm_fails(tmp_path):
    script = _read_script()
    runtime_start = script.index("# ROS setup reads")
    dispatch_start = script.index(
        'if [[ "$ALL" == "1" ]]', script.index("run_one() {"))
    stubbed = (
        script[:runtime_start]
        + 'run_one() { [[ "$1" != "controlled_fixed_028" ]]; }\n\n'
        + script[dispatch_start:]
    )
    runner = tmp_path / "runner.sh"
    runner.write_text(stubbed, encoding="utf-8")
    env = dict(os.environ)
    env.pop("DRY_RUN", None)

    result = subprocess.run(
        ["bash", str(runner), "--all", "--out", str(tmp_path / "out")],
        cwd=str(REPO_ROOT),
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "RUN_FAIL controlled_fixed_028" in result.stdout
