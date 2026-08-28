from gazebo_sim.nodes.field_dropout_controller import PositionDropoutWindow
from vgr_core.safety import Pose


def test_decreasing_x_window_transitions_exactly_once():
    core = PositionDropoutWindow(enabled=True, dropout_x=1.25, resume_x=0.70)
    assert core.observe(Pose(1.30, -0.2, 0.0), 1.0) is None
    start = core.observe(Pose(1.24, -0.2, 0.0), 2.0)
    assert start is not None and start.dropout is True
    core.commit(start)
    assert core.observe(Pose(1.00, -0.2, 0.0), 3.0) is None
    end = core.observe(Pose(0.69, -0.2, 0.0), 4.0)
    assert end is not None and end.dropout is False
    core.commit(end)
    assert core.observe(Pose(0.50, -0.2, 0.0), 5.0) is None
    assert core.observe(Pose(1.40, -0.2, 0.0), 6.0) is None


def test_disabled_window_never_requests_gate_change():
    core = PositionDropoutWindow(enabled=False, dropout_x=1.25, resume_x=0.70)
    assert core.observe(Pose(0.0, 0.0, 0.0), 1.0) is None
