from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
HEADLESS_RUNNER = REPO_ROOT / "gazebo_sim" / "scripts" / "run_gs_scenario.sh"
GUI_RUNNER = REPO_ROOT / "gazebo_sim" / "scripts" / "run_gs_scenario_gui.sh"
SMOKE_RUNNER = REPO_ROOT / "gazebo_sim" / "scripts" / "run_g4_vision_smoke.sh"
ACCURACY_RUNNER = REPO_ROOT / "gazebo_sim" / "scripts" / "measure_vision_accuracy.sh"


def _read(path: Path) -> str:
    assert path.exists(), f"missing script: {path}"
    return path.read_text(encoding="utf-8")


def test_gs_runners_accept_optional_pose_source_and_branch_to_vision_stack():
    for path in (HEADLESS_RUNNER, GUI_RUNNER):
        text = _read(path)
        assert "POSE_SOURCE=\"${3:-pseudo}\"" in text
        assert "pseudo|vision" in text
        assert "gazebo_sim.nodes.aruco_detector" in text
        assert "gazebo_sim.nodes.pseudo_aruco" in text
        assert "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image" in text
        assert "vision 模式" in text


def test_headless_runner_enables_headless_rendering_only_for_vision_and_skips_dropout():
    text = _read(HEADLESS_RUNNER)
    assert "--headless-rendering" in text
    assert 'if [ "$POSE_SOURCE" = "vision" ]' in text
    assert 'WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena.world"' in text
    assert 'if [ "$POSE_SOURCE" = "vision" ]; then' in text
    assert 'WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena_vision.world"' in text
    assert "GS2 vision 模式跳過 marker dropout service" in text
    assert "/aruco/set_dropout" in text
    assert 'SPAWN_POSE="${VGR_SPAWN_POSE:-0.5 0 0 0 0 0}"' in text


def test_gui_runner_uses_vision_world_only_for_vision_pose_source():
    text = _read(GUI_RUNNER)
    assert 'WORLD_SRC="$REPO/gazebo_sim/worlds/vgr_arena.world"' in text
    assert 'if [ "$POSE_SOURCE" = "vision" ]; then' in text
    assert 'WORLD_SRC="$REPO/gazebo_sim/worlds/vgr_arena_vision.world"' in text
    assert 'pathlib.Path(sys.argv[1]).read_text()' in text


def test_g4_vision_smoke_has_required_stage_markers_and_success_sentinel():
    text = _read(SMOKE_RUNNER)
    for marker in ("[G4-1]", "[G4-2]", "[G4-3]", "[G4-4]"):
        assert marker in text
    assert "G4_SMOKE_OK" in text
    assert "--headless-rendering" in text
    assert "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image" in text
    assert "gazebo_sim.nodes.aruco_detector" in text
    assert 'WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena_vision.world"' in text
    assert "run_gs_scenario.sh\" GS2 clamp_watchdog vision" in text


def test_g4_vision_smoke_isolates_runtime_and_uses_near_accuracy_spawn():
    text = _read(SMOKE_RUNNER)
    assert 'export ROS_DOMAIN_ID=$((RANDOM % 100 + 100))' in text
    assert 'export IGN_PARTITION="g4smoke_$$"' in text
    assert "跨執行殘留程序" in text
    assert "2.5 0 0 0 0 0" in text
    assert "0.5 0 0 0 0 0" not in text
    assert "ps -p" in text
    assert "awk" in text
    assert "pkill -f" not in text


def test_vision_accuracy_measurement_script_contract():
    text = _read(ACCURACY_RUNNER)
    assert "vision_accuracy_${STAMP}.md" in text
    assert "3.0 2.5 2.0 1.5 1.0 0.5" in text
    assert "ArucoWorldLocalizer" in text
    assert "locate(image_bgr)" in text
    assert "| 距離 m | 偵測 ID | 位姿誤差 m |" in text
    assert 'export ROS_DOMAIN_ID=$((RANDOM % 100 + 100))' in text
    assert 'export IGN_PARTITION="g4measure_$$"' in text
    assert "ps -p" in text
    assert "awk" in text
    assert "pkill -f" not in text
