"""固定步長主迴圈：nav → filter → link → plant，輸出完整 Trace。

控制迴圈（預設 20 Hz）與 plant 積分（預設 100 Hz）分離。
固定亂數種子與固定步長，同一 (scenario, filter) 每次結果完全相同。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from vgr_core.motion import DiffDriveParams

from .link import CommandLink
from .scenario import Scenario
from .sensors import ArucoLocalizer
from .types import (Observation, Pose, SafetyDecision, SafetyFilter,
                    StaticInfo, Twist)
from .vehicle import DiffDriveVehicle
from .world import World


@dataclass(frozen=True)
class TraceSample:
    t: float
    true_pose: Pose                 # ground truth，只給 metrics/繪圖
    est_pose: Pose | None           # 安全層實際看到的位姿
    pose_age_s: float
    link_age_s: float
    desired: Twist                  # nav 想要的
    cmd: Twist                      # filter 放行的
    mode: str
    actual_twist: Twist             # 車體實際速度
    clearance: float                # ground truth 淨空
    debug: dict[str, float]


@dataclass
class Trace:
    scenario_name: str
    filter_name: str
    world: World
    samples: list[TraceSample]


def run_scenario(scenario: Scenario, filt: SafetyFilter) -> Trace:
    world = scenario.make_world()
    nav = scenario.make_nav()
    vehicle = DiffDriveVehicle(DiffDriveParams(), pose=scenario.start_pose,
                               **scenario.vehicle_kwargs)
    localizer = ArucoLocalizer(**scenario.localizer_kwargs)
    link = CommandLink(timeout_s=scenario.link_timeout_s)

    filt.reset(StaticInfo(
        params=vehicle.params,
        robot_radius_m=scenario.robot_radius_m,
        geofence=world.geofence,
        max_v_mps=scenario.max_v_mps,
        max_omega_rad_s=scenario.max_omega_rad_s,
    ))

    control_dt = 1.0 / scenario.control_hz
    plant_dt = 1.0 / scenario.plant_hz
    substeps = max(1, round(control_dt / plant_dt))
    ticks = round(scenario.duration_s * scenario.control_hz)

    samples: list[TraceSample] = []
    for i in range(ticks):
        t = i * control_dt

        est_pose, pose_age = localizer.observe(
            vehicle.pose, t, dropout=scenario.faults.active("aruco_dropout", t))
        obs = Observation(
            pose=est_pose,
            pose_age_s=pose_age,
            wheel_feedback=vehicle.wheel_counts_per_s,
            obstacles=world.obstacles,
            link_age_s=link.age_s(t),
            goal=world.goal,
            goal_age_s=0.0 if world.goal is not None else math.inf,
        )

        desired = nav.command(obs, t)
        decision: SafetyDecision = filt.filter(desired, obs, t, control_dt)

        link.send(decision.cmd, t, dropped=scenario.faults.active("link_drop", t))
        link.poll(vehicle, t)

        for _ in range(substeps):
            vehicle.step(plant_dt)

        samples.append(TraceSample(
            t=t,
            true_pose=vehicle.pose,
            est_pose=est_pose,
            pose_age_s=pose_age,
            link_age_s=obs.link_age_s,
            desired=desired,
            cmd=decision.cmd,
            mode=decision.mode,
            actual_twist=vehicle.twist_actual,
            clearance=world.min_clearance(vehicle.pose),
            debug=decision.debug,
        ))

    return Trace(scenario_name=scenario.name, filter_name=filt.name,
                 world=world, samples=samples)
