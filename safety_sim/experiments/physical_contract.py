"""Data contracts and randomized schedules for the R1/R3 experiments."""
from __future__ import annotations

import enum
import hashlib
import json
import random
import re
from dataclasses import asdict, dataclass
from pathlib import Path


# --- Fixed policy values ---
ROBOT_RADIUS_M = 0.23
CLEARANCE_M = 0.05
D_SAFE_M = 0.28
BLIND_BUDGET_M = 2.0
BLIND_MAX_S = 60.0
POSE_FRESH_S = 0.40
LINK_FRESH_S = 0.50
GOAL_FRESH_S = 0.50
PLATFORM_CEILING_MPS = 0.245044226980  # 900 counts/s / 750 * pi * 0.065
R1_SPEEDS_MPS = (0.05, 0.15, 0.22)
R3_SPEED_MPS = 0.15
R1_MAX_SPEED_MPS = 0.22

# --- R1 matrix ---
R1_CELLS = (
    (0.5, 0.15), (1.0, 0.15), (2.0, 0.15), (3.0, 0.15),
    (1.0, 0.05), (1.0, 0.22), (2.0, 0.22), (3.0, 0.22),
)
R1_BLOCKS = 10
R1_ZERO_REPS = 10
R1_MOVING_RUNS = R1_BLOCKS * len(R1_CELLS)   # 80
R1_TOTAL = R1_MOVING_RUNS + R1_ZERO_REPS     # 90

# --- R3 matrix ---
R3_SAPF_RUNS = 10
R3_PASSTHROUGH_RUNS = 3
R3_TOTAL = R3_SAPF_RUNS + R3_PASSTHROUGH_RUNS  # 13


class StartCalibrationState(str, enum.Enum):
    FRESH_VISUAL_ANCHOR = "fresh_visual_anchor"
    CONTINUOUS_VISUAL = "continuous_visual"
    BLIND_AFTER_DISTANCE = "blind_after_distance"


class RunValidity(str, enum.Enum):
    VALID = "valid"
    INVALID = "invalid"      # 預先宣告的執行/儀器錯誤
    SMOKE = "smoke"          # 非正式，永不進入正式統計


class InvalidReason(str, enum.Enum):
    MISSING_REQUIRED_FIELD = "missing_required_field"
    WRONG_START_CONDITION = "wrong_start_condition"
    WRONG_METHOD_OR_HASH = "wrong_method_or_hash"
    GROUND_TRUTH_CALIBRATION_FAILED = "ground_truth_calibration_failed"
    HUMAN_OR_CABLE_INTERFERENCE = "human_or_cable_interference"
    LOGGING_FAILURE = "logging_failure"
    UNRELATED_ESTOP = "unrelated_estop"


def _run_id(experiment: str, index: int, seed: int) -> str:
    h = hashlib.sha1(f"{experiment}:{index}:{seed}".encode()).hexdigest()[:10]
    return f"{experiment}_{index:03d}_{h}"


@dataclass(frozen=True)
class RunManifest:
    """Validated manifest; missing required field groups are rejected."""

    # 6.1 identity
    experiment: str          # "R1" | "R3"
    scenario: str
    run_id: str
    utc_timestamp: str
    method: str
    block_id: int
    replicate_index: int
    planned_repetitions: int
    seed: int
    code_revision: str
    dirty_flag: bool
    runtime_env: str
    risk_hash: str
    calibration_hash: str
    scenario_geometry_hash: str
    operator: str
    estop_operator: str
    # 6.2 speed
    commanded_speed_mps: float
    speed_ceiling_mps: float
    # 6.3 blind travel
    target_blind_distance_m: float
    start_calibration_state: StartCalibrationState
    # 6.4 baseline
    baseline_length_m: float
    measurement_instrument: str
    baseline_residual_m: float
    # 6.5 load and floor
    payload_kg: float
    payload_config: str
    floor_material: str
    floor_condition: str
    # 6.6 start calibration
    start_pose: tuple[float, float, float] | None = None
    start_pose_age_s: float | None = None
    accepted_correction_id: str | None = None
    start_jig_residual_m: float | None = None

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(data: dict) -> "RunManifest":
        data = dict(data)
        data["start_calibration_state"] = StartCalibrationState(
            data["start_calibration_state"])
        data["start_pose"] = tuple(data["start_pose"]) if data.get("start_pose") else None
        return RunManifest(**data)


# 6.x 必填欄位群組：缺一即 invalid。
_REQUIRED_GROUPS = {
    "identity": ("experiment", "scenario", "run_id", "utc_timestamp", "method",
                 "block_id", "replicate_index", "planned_repetitions", "seed",
                 "code_revision", "dirty_flag", "runtime_env", "risk_hash",
                 "calibration_hash", "scenario_geometry_hash", "operator",
                 "estop_operator"),
    "speed": ("commanded_speed_mps", "speed_ceiling_mps"),
    "blind": ("target_blind_distance_m", "start_calibration_state"),
    "baseline": ("baseline_length_m", "measurement_instrument",
                 "baseline_residual_m"),
    "load_floor": ("payload_kg", "payload_config", "floor_material",
                   "floor_condition"),
}


def validate_manifest(manifest: RunManifest) -> None:
    """Raise ValueError when any required field group is incomplete."""
    d = manifest.to_dict()
    missing: list[str] = []
    for group, fields in _REQUIRED_GROUPS.items():
        for f in fields:
            if f not in d or d[f] is None or d[f] == "":
                missing.append(f"{group}:{f}")
    if missing:
        raise ValueError(f"manifest missing required fields: {sorted(missing)}")
    if manifest.commanded_speed_mps > manifest.speed_ceiling_mps + 1e-9:
        raise ValueError(
            f"commanded speed {manifest.commanded_speed_mps} exceeds ceiling "
            f"{manifest.speed_ceiling_mps}")


@dataclass(frozen=True)
class RunResult:
    run_id: str
    validity: RunValidity
    invalid_reason: InvalidReason | None = None
    # 6.7 outcome
    true_min_clearance_m: float | None = None
    collided: bool | None = None
    crossed_line: bool | None = None
    reached_goal: bool | None = None
    timeout: bool | None = None
    interruption_count: int | None = None
    manual_intervention: bool | None = None
    intervention_reason: str | None = None
    # 量測
    actual_odom_blind_m: float | None = None
    actual_mean_speed_mps: float | None = None
    actual_p95_speed_mps: float | None = None
    actual_max_speed_mps: float | None = None
    # Continuous-vision appendix; excluded from b/k fitting.
    appendix_continuous_visual_error_m: float | None = None


def _serialize(obj) -> object:
    if isinstance(obj, enum.Enum):
        return obj.value
    if isinstance(obj, tuple):
        return list(obj)
    raise TypeError(type(obj))


def atomic_write_new_json(path: Path | str, payload: dict) -> None:
    """Refuse to overwrite an existing run artifact."""
    path = Path(path)
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2,
                              default=_serialize), encoding="utf-8")
    tmp.replace(path)


def artifact_checksum(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


@dataclass(frozen=True)
class PlannedRun:
    index: int
    run_id: str
    experiment: str
    scenario: str
    method: str
    block_id: int
    replicate_index: int
    target_blind_distance_m: float
    commanded_speed_mps: float
    start_calibration_state: StartCalibrationState


def build_r1_schedule(seed: int, *, runout_max_m: float = 3.0) -> tuple[tuple[PlannedRun, ...], dict]:
    """Build 10 randomized blocks with eight moving cells per block.

    If runout is shorter than 3 m, omit the 3 m at 0.22 m/s cell and record it.
    """
    if runout_max_m < 3.0:
        cells = [c for c in R1_CELLS if not (c[0] == 3.0 and c[1] == 0.22)]
        dropped = {"3.0_m_0.22_mps": True}
    else:
        cells = list(R1_CELLS)
        dropped = {}
    rng = random.Random(f"R1:{seed}")
    runs: list[PlannedRun] = []
    for block in range(R1_BLOCKS):
        order = list(range(len(cells)))
        rng.shuffle(order)
        for rep, ci in enumerate(order):
            dist, speed = cells[ci]
            runs.append(PlannedRun(
                index=len(runs), run_id=_run_id("R1", len(runs), seed),
                experiment="R1", scenario=f"blind_{dist:g}m_{speed:g}mps",
                method="calibration", block_id=block, replicate_index=rep,
                target_blind_distance_m=dist, commanded_speed_mps=speed,
                start_calibration_state=StartCalibrationState.FRESH_VISUAL_ANCHOR,
            ))
    for z in range(R1_ZERO_REPS):
        runs.append(PlannedRun(
            index=len(runs), run_id=_run_id("R1", len(runs), seed),
            experiment="R1", scenario="blind_0m_0mps",
            method="calibration_zero", block_id=-1, replicate_index=z,
            target_blind_distance_m=0.0, commanded_speed_mps=0.0,
            start_calibration_state=StartCalibrationState.FRESH_VISUAL_ANCHOR,
        ))
    return tuple(runs), dropped


def build_r3_schedule(seed: int) -> tuple[PlannedRun, ...]:
    """Randomize ten SAPF-new and three passthrough runs."""
    rng = random.Random(f"R3:{seed}")
    runs: list[PlannedRun] = []
    for method, count in (("sapf_new", R3_SAPF_RUNS),
                          ("passthrough", R3_PASSTHROUGH_RUNS)):
        for rep in range(count):
            runs.append(PlannedRun(
                index=len(runs), run_id=_run_id("R3", len(runs), seed),
                experiment="R3", scenario="geofence_virtual_line", method=method,
                block_id=0, replicate_index=rep,
                target_blind_distance_m=0.0, commanded_speed_mps=R3_SPEED_MPS,
                start_calibration_state=StartCalibrationState.CONTINUOUS_VISUAL,
            ))
    rng.shuffle(runs)
    return tuple(runs)


def write_schedule(path: Path, seed: int, runs: tuple[PlannedRun, ...],
                   meta: dict | None = None) -> None:
    payload = {
        "seed": seed,
        "count": len(runs),
        "meta": meta or {},
        "runs": [r.__dict__ for r in runs],
    }
    atomic_write_new_json(path, payload)


def load_schedule(path: Path) -> tuple[int, tuple[PlannedRun, ...]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    runs = tuple(PlannedRun(**r) for r in data["runs"])
    return int(data["seed"]), runs


def _slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", value).strip("_")


def run_dir_for(root: Path, experiment: str, run_id: str) -> Path:
    return root / _slug(experiment) / run_id
