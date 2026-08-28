"""S8：單一靜態障礙擋在起點與 goal 之間（SAPF 繞行驗證情境）。

S8 的 goal 與障礙是 GS3 Gazebo 場景的單一資料來源：`SAPF_OBSTACLE` 同時
餵給 safety_sim 的 World 與 Gazebo world 產生器，避免兩邊幾何不一致。

授權來源用 constant-forward ScriptedNav（與 S1/S2 一致）：SAPF 是 planner，
自己決定轉向與速度；若用 WaypointNav，它在車頭偏離 goal 超過 90° 時輸出
v=0，會把 SAPF 的繞行弧線誤判成「任務取消」。到達 goal 由 filter 的
goal_reached 停止負責。
"""
from __future__ import annotations

from vgr_core.geometry.arena_geometry import Box2D

from ..nav import ScriptedNav
from ..scenario import DEFAULT_ROBOT_RADIUS_M, Expectation, Scenario
from ..types import Circle, Twist
from ..world import World
from .basic import ARENA

# 矩形障礙 (2.0, 0.0)、0.40×0.40 m：起點 (0.5, 0) 到 goal (3.2, 0) 的直線
# 必然穿過障礙（含 0.23 m 車體半徑的膨脹包絡），因此成功抵達就代表繞行。
# 矩形（非圓柱）：平直邊與角點讓繞行需沿邊走、過角切換，比圓柱切點更嚴格。
# 0.40×0.40：與圓柱直徑同寬、通道一致（膨脹後 y∈[-0.43,0.43]，通道 0.34 m）；
# 0.60 高在 2 m 場地通道只剩 0.24 m（比車窄），不可通行。
SAPF_OBSTACLE = Box2D(x=2.0, y=0.0, size_x=0.40, size_y=0.40)
S8_GOAL = (3.2, 0.0)


def _make_s8_world() -> World:
    return World(geofence=ARENA, obstacles=(SAPF_OBSTACLE,),
                 robot_radius_m=DEFAULT_ROBOT_RADIUS_M, goal=S8_GOAL)


def make_s8_single_obstacle_detour() -> Scenario:
    """S8：前向命令持續授權，矩形擋在中途。考 SAPF 繞行與保持距離。

    矩形（0.40×0.60）比圓柱更嚴格：平直邊與角點使 vortex 在角落方向衝突，
    過角需 ~50s（圓柱 ~20s），故 duration 120s。
    """
    return Scenario(
        name="S8",
        description="單障礙繞行：goal 在矩形正後方，必須偏離直線再回到 goal",
        make_world=_make_s8_world,
        make_nav=lambda: ScriptedNav(((0.0, Twist(0.15, 0.0)),)),
        duration_s=120.0,
        expectation=Expectation(require_no_collision=True,
                                max_final_goal_distance_m=0.15),
    )
