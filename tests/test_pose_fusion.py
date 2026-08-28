"""PoseFuser unit tests."""
import math

from ros2_ws.src.vgr_safety_gate.vgr_safety_gate.pose_fusion import (
    FusedPose, PoseFuser)

DT = 0.05


def _true_map_pose(odom, offset=(1.0, 0.5, 0.0)):
    """真實 map 位姿 = odom 平移 offset（無旋轉情境）。"""
    return (odom[0] + offset[0], odom[1] + offset[1], odom[2] + offset[2])


def _drive(fuser, *, seconds, v=0.10, vision_every=None, vision_fn=None,
           t0=0.0, y0=0.0):
    """odom 沿 +y 直行；可選每 vision_every 秒餵一筆視覺（vision_fn 產生）。
    回傳 (t, y)。"""
    t, y = t0, y0
    steps = int(seconds / DT)
    for i in range(steps):
        t += DT
        y += v * DT
        fuser.update_odom(0.0, y, math.pi / 2, t)
        if vision_every and vision_fn and i % int(vision_every / DT) == 0:
            vision_fn(fuser, t, y)
    return t, y


def test_converges_to_constant_offset():
    fuser = PoseFuser()
    offset = (1.0, 0.5, 0.0)

    def feed(f, t, y):
        truth = _true_map_pose((0.0, y, math.pi / 2), offset)
        f.update_vision(*truth, stamp_s=t)

    t, y = _drive(fuser, seconds=10.0, vision_every=0.2, vision_fn=feed)
    est = fuser.estimate(t)
    truth = _true_map_pose((0.0, y, math.pi / 2), offset)
    assert est is not None
    assert math.hypot(est.x - truth[0], est.y - truth[1]) < 0.01
    assert abs(est.yaw - truth[2]) < math.radians(0.5)


def test_time_alignment_beats_naive_pairing():
    """視覺內容與時間戳均延遲 0.3s：時間對齊後誤差 <1.5cm。
    （拿到達時 odom 配對會固定錯 v*delay = 3cm。）"""
    fuser = PoseFuser()
    offset = (1.0, 0.5, 0.0)
    delay = 0.3
    history: list[tuple[float, float]] = []   # (t, y)

    t, y = 0.0, 0.0
    for i in range(int(10.0 / DT)):
        t += DT
        y += 0.10 * DT
        fuser.update_odom(0.0, y, math.pi / 2, t)
        history.append((t, y))
        if i % int(0.2 / DT) == 0 and t - delay > 0.2:
            # 找 0.3s 前的真實狀態
            t_meas = t - delay
            y_meas = min(history, key=lambda h: abs(h[0] - t_meas))[1]
            truth = _true_map_pose((0.0, y_meas, math.pi / 2), offset)
            fuser.update_vision(*truth, stamp_s=t_meas)
    est = fuser.estimate(t)
    truth_now = _true_map_pose((0.0, y, math.pi / 2), offset)
    assert est is not None
    err = math.hypot(est.x - truth_now[0], est.y - truth_now[1])
    assert err < 0.015, f"時間對齊失效: {err:.3f}m"


def test_flip_outliers_rejected():
    fuser = PoseFuser()
    offset = (1.0, 0.5, 0.0)

    def feed(f, t, y):
        truth = _true_map_pose((0.0, y, math.pi / 2), offset)
        f.update_vision(*truth, stamp_s=t)

    t, y = _drive(fuser, seconds=5.0, vision_every=0.2, vision_fn=feed)

    for i in range(20):
        t += DT
        y += 0.10 * DT
        fuser.update_odom(0.0, y, math.pi / 2, t)
        truth = _true_map_pose((0.0, y, math.pi / 2), offset)
        if i % 2 == 1:  # 翻解：整筆偏 0.2m
            ok = fuser.update_vision(truth[0] + 0.2, truth[1], truth[2], stamp_s=t)
            assert not ok, "翻解樣本必須被拒收"
        else:
            fuser.update_vision(*truth, stamp_s=t)
        est = fuser.estimate(t)
        err = math.hypot(est.x - truth[0], est.y - truth[1])
        assert err < 0.02, f"翻解污染了估計: {err:.3f}m"


def test_vision_loss_degrades_to_odom():
    fuser = PoseFuser()
    offset = (1.0, 0.5, 0.0)

    def feed(f, t, y):
        truth = _true_map_pose((0.0, y, math.pi / 2), offset)
        f.update_vision(*truth, stamp_s=t)

    t, y = _drive(fuser, seconds=5.0, vision_every=0.2, vision_fn=feed)
    est_before = fuser.estimate(t)
    # 視覺全斷，再走 1.0m
    t, y = _drive(fuser, seconds=10.0, t0=t, y0=y)
    est = fuser.estimate(t)
    truth = _true_map_pose((0.0, y, math.pi / 2), offset)
    assert est is not None
    # 修正量凍結 → 估計 = 修正 ∘ 最新 odom，應精確跟隨 odom
    assert math.hypot(est.x - truth[0], est.y - truth[1]) < 1e-6
    # 1.0m 盲走＋最後一筆視覺到視覺死亡之間殘餘的 ≤0.02m
    assert 1.0 - 1e-6 <= est.drift_m <= 1.02
    assert est.corr_age_s > est_before.corr_age_s
    assert est.corr_age_s > 9.9


def test_relocalization_after_persistent_rejects():
    fuser = PoseFuser(reloc_after_rejects=20)
    offset_old = (1.0, 0.5, 0.0)
    offset_new = (1.5, 0.5, 0.0)   # 修正量跳 0.5m（車被搬動）

    def feed_old(f, t, y):
        truth = _true_map_pose((0.0, y, math.pi / 2), offset_old)
        f.update_vision(*truth, stamp_s=t)

    t, y = _drive(fuser, seconds=5.0, vision_every=0.2, vision_fn=feed_old)

    accepted_at = None
    for i in range(80):
        t += DT
        fuser.update_odom(0.0, y, math.pi / 2, t)
        truth = _true_map_pose((0.0, y, math.pi / 2), offset_new)
        ok = fuser.update_vision(*truth, stamp_s=t)
        if ok and accepted_at is None:
            accepted_at = i
    assert accepted_at is not None, "重定位永遠沒被接受"
    assert accepted_at >= 19, f"太早接受（第 {accepted_at} 筆）＝一致性門失效"
    # 再收斂幾筆
    for _ in range(40):
        t += DT
        fuser.update_odom(0.0, y, math.pi / 2, t)
        truth = _true_map_pose((0.0, y, math.pi / 2), offset_new)
        fuser.update_vision(*truth, stamp_s=t)
    est = fuser.estimate(t)
    truth = _true_map_pose((0.0, y, math.pi / 2), offset_new)
    assert math.hypot(est.x - truth[0], est.y - truth[1]) < 0.02
