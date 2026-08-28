"""產生 VGR 差速車的 Gazebo Fortress SDF。"""
from __future__ import annotations

import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

from vgr_core.motion import DiffDriveParams


DEFAULT_OUTPUT = "gazebo_sim/models/vgr_diff_drive/model.sdf"
BODY_MASS_KG = 1.5
BODY_SIZE_M = (0.40, 0.22, 0.065)  # representative public robot envelope
SDF_VERSION = "1.8"
CAMERA_NAME = "front_camera"
CAMERA_TOPIC = "/camera/image_raw"
CAMERA_WIDTH_PX = 640
CAMERA_HEIGHT_PX = 480
CAMERA_HORIZONTAL_FOV_RAD = 1.2
CAMERA_UPDATE_RATE_HZ = 15.0
CAMERA_HEIGHT_M = 0.17
CAMERA_FRONT_X_M = BODY_SIZE_M[0] / 2.0
CASTER_OFFSET_FROM_REAR_AXLE_M = 0.135
CASTER_Y_OFFSET_M = 0.045

# Gazebo Fortress / ignition-gazebo6 使用 ignition::* system 名稱與
# libignition-gazebo-*.so 檔名；Garden / gz-sim7+ 改為 gz::sim::* 與
# gz-sim-* 命名。將來升級 Gazebo 世代時，只需集中調整這些常數。
DIFF_DRIVE_PLUGIN_FILENAME = "libignition-gazebo-diff-drive-system.so"
DIFF_DRIVE_PLUGIN_NAME = "ignition::gazebo::systems::DiffDrive"
ODOMETRY_PUBLISHER_PLUGIN_FILENAME = "libignition-gazebo-odometry-publisher-system.so"
ODOMETRY_PUBLISHER_PLUGIN_NAME = "ignition::gazebo::systems::OdometryPublisher"
JOINT_STATE_PLUGIN_FILENAME = "libignition-gazebo-joint-state-publisher-system.so"
JOINT_STATE_PLUGIN_NAME = "ignition::gazebo::systems::JointStatePublisher"
TRUE_POSE_TOPIC = "/sim/true_pose"


def _add_text(parent: ET.Element, tag: str, text: object) -> ET.Element:
    elem = ET.SubElement(parent, tag)
    elem.text = str(text)
    return elem


def _fmt(value: float) -> str:
    return f"{value:.12g}"


def _pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> str:
    return " ".join(_fmt(part) for part in (x, y, z, roll, pitch, yaw))


def _box_inertia(mass_kg: float, x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    ixx = mass_kg * (y_m * y_m + z_m * z_m) / 12.0
    iyy = mass_kg * (x_m * x_m + z_m * z_m) / 12.0
    izz = mass_kg * (x_m * x_m + y_m * y_m) / 12.0
    return ixx, iyy, izz


def _cylinder_inertia_y_axis(mass_kg: float, radius_m: float, length_m: float) -> tuple[float, float, float]:
    i_axis = 0.5 * mass_kg * radius_m * radius_m
    i_perp = mass_kg * (3.0 * radius_m * radius_m + length_m * length_m) / 12.0
    return i_perp, i_axis, i_perp


def _sphere_inertia(mass_kg: float, radius_m: float) -> tuple[float, float, float]:
    inertia = 2.0 * mass_kg * radius_m * radius_m / 5.0
    return inertia, inertia, inertia


def _add_inertial(parent: ET.Element, mass_kg: float, inertia: tuple[float, float, float]) -> None:
    inertial = ET.SubElement(parent, "inertial")
    _add_text(inertial, "mass", _fmt(mass_kg))
    inertia_elem = ET.SubElement(inertial, "inertia")
    ixx, iyy, izz = inertia
    _add_text(inertia_elem, "ixx", _fmt(ixx))
    _add_text(inertia_elem, "ixy", "0")
    _add_text(inertia_elem, "ixz", "0")
    _add_text(inertia_elem, "iyy", _fmt(iyy))
    _add_text(inertia_elem, "iyz", "0")
    _add_text(inertia_elem, "izz", _fmt(izz))


def _add_box_geometry(parent: ET.Element, size: tuple[float, float, float]) -> None:
    geometry = ET.SubElement(parent, "geometry")
    box = ET.SubElement(geometry, "box")
    _add_text(box, "size", " ".join(_fmt(part) for part in size))


def _add_cylinder_geometry(parent: ET.Element, radius_m: float, length_m: float) -> None:
    geometry = ET.SubElement(parent, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    _add_text(cylinder, "radius", _fmt(radius_m))
    _add_text(cylinder, "length", _fmt(length_m))


def _add_sphere_geometry(parent: ET.Element, radius_m: float) -> None:
    geometry = ET.SubElement(parent, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    _add_text(sphere, "radius", _fmt(radius_m))


def _add_material(visual: ET.Element, rgba: str) -> None:
    """GUI 辨識用的簡單單色材質；rgba 如 "0.1 0.35 0.8 1"。"""
    material = ET.SubElement(visual, "material")
    _add_text(material, "ambient", rgba)
    _add_text(material, "diffuse", rgba)


def _add_box_link(model: ET.Element, name: str, pose: str, size: tuple[float, float, float], mass_kg: float) -> None:
    link = ET.SubElement(model, "link", {"name": name})
    _add_text(link, "pose", pose)
    _add_inertial(link, mass_kg, _box_inertia(mass_kg, *size))
    collision = ET.SubElement(link, "collision", {"name": f"{name}_collision"})
    _add_box_geometry(collision, size)
    visual = ET.SubElement(link, "visual", {"name": f"{name}_visual"})
    _add_box_geometry(visual, size)
    _add_material(visual, "0.1 0.35 0.8 1")   # 車體藍，GUI 一眼可辨


def _add_camera_sensor(chassis_link: ET.Element, front_x_m: float) -> None:
    """Add a front-mounted camera using the public model constants."""
    sensor = ET.SubElement(chassis_link, "sensor", {"name": CAMERA_NAME, "type": "camera"})
    _add_text(sensor, "pose", _pose(front_x_m, 0.0, CAMERA_HEIGHT_M))
    _add_text(sensor, "topic", CAMERA_TOPIC)
    _add_text(sensor, "update_rate", _fmt(CAMERA_UPDATE_RATE_HZ))
    _add_text(sensor, "always_on", "1")
    _add_text(sensor, "visualize", "0")
    camera = ET.SubElement(sensor, "camera")
    _add_text(camera, "horizontal_fov", _fmt(CAMERA_HORIZONTAL_FOV_RAD))
    image = ET.SubElement(camera, "image")
    _add_text(image, "width", CAMERA_WIDTH_PX)
    _add_text(image, "height", CAMERA_HEIGHT_PX)
    _add_text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    _add_text(clip, "near", "0.02")
    _add_text(clip, "far", "10")


def _add_wheel_link(
    model: ET.Element,
    name: str,
    pose: str,
    radius_m: float,
    width_m: float,
    mass_kg: float,
    *,
    friction_mu: float | None = None,
    friction_mu2: float | None = None,
) -> None:
    link = ET.SubElement(model, "link", {"name": name})
    _add_text(link, "pose", pose)
    _add_inertial(link, mass_kg, _cylinder_inertia_y_axis(mass_kg, radius_m, width_m))
    for tag in ("collision", "visual"):
        elem = ET.SubElement(link, tag, {"name": f"{name}_{tag}"})
        _add_text(elem, "pose", _pose(0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0))
        _add_cylinder_geometry(elem, radius_m, width_m)
        if tag == "visual":
            _add_material(elem, "0.05 0.05 0.05 1")   # 輪胎黑
    if friction_mu is not None:
        collision = next(c for c in link.iter("collision"))
        surface = ET.SubElement(collision, "surface")
        friction = ET.SubElement(surface, "friction")
        ode = ET.SubElement(friction, "ode")
        _add_text(ode, "mu", f"{friction_mu}")
        if friction_mu2 is not None:
            _add_text(ode, "mu2", f"{friction_mu2}")


def _add_caster_link(model: ET.Element, name: str, pose: str, radius_m: float, mass_kg: float) -> None:
    link = ET.SubElement(model, "link", {"name": name})
    _add_text(link, "pose", pose)
    _add_inertial(link, mass_kg, _sphere_inertia(mass_kg, radius_m))
    collision = ET.SubElement(link, "collision", {"name": f"{name}_collision"})
    _add_sphere_geometry(collision, radius_m)
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    _add_text(ode, "mu", "0.001")
    _add_text(ode, "mu2", "0.001")
    visual = ET.SubElement(link, "visual", {"name": f"{name}_visual"})
    _add_sphere_geometry(visual, radius_m)
    _add_material(visual, "0.6 0.6 0.6 1")   # 萬向輪灰


def _add_revolute_joint(model: ET.Element, name: str, child: str) -> None:
    joint = ET.SubElement(model, "joint", {"name": name, "type": "revolute"})
    _add_text(joint, "parent", "chassis")
    _add_text(joint, "child", child)
    axis = ET.SubElement(joint, "axis")
    _add_text(axis, "xyz", "0 1 0")


def _add_fixed_joint(model: ET.Element, name: str, child: str) -> None:
    joint = ET.SubElement(model, "joint", {"name": name, "type": "fixed"})
    _add_text(joint, "parent", "chassis")
    _add_text(joint, "child", child)


def max_linear_velocity_mps(params: DiffDriveParams) -> float:
    """由 firmware counts/s 上限換算 Gazebo diff_drive 的線速度上限。"""
    circumference_m = math.pi * params.wheel_diameter_m
    counts_per_rev = max(params.left_counts_per_rev, params.right_counts_per_rev)
    return params.max_counts_per_s / counts_per_rev * circumference_m


def build_camera_info() -> dict[str, float | int | str]:
    """由 SDF camera 參數計算 pinhole 內參。"""
    fx = CAMERA_WIDTH_PX / (2.0 * math.tan(CAMERA_HORIZONTAL_FOV_RAD / 2.0))
    return {
        "width": CAMERA_WIDTH_PX,
        "height": CAMERA_HEIGHT_PX,
        "horizontal_fov_rad": CAMERA_HORIZONTAL_FOV_RAD,
        "fx": fx,
        "fy": fx,
        "cx": CAMERA_WIDTH_PX / 2.0,
        "cy": CAMERA_HEIGHT_PX / 2.0,
        "topic": CAMERA_TOPIC,
    }


def build_robot_sdf(
    params: DiffDriveParams,
    model_name: str = "vgr_diff_drive",
    *,
    left_wheel_mu: float | None = None,
    right_wheel_mu: float | None = None,
    wheel_mu2: float | None = None,
    left_wheel_radius_m: float | None = None,
    right_wheel_radius_m: float | None = None,
) -> str:
    """回傳差速車 SDF XML 字串。

    Wheel geometry comes from `DiffDriveParams`; body, camera, and caster
    placement use the representative public model constants above.

    `left_wheel_mu`/`right_wheel_mu`（及共用 `wheel_mu2`）可對驅動輪加入
    ODE friction：低摩擦輪會側滑，產生橫向漂移（R1 Gazebo 實驗）。

    `left_wheel_radius_m`/`right_wheel_radius_m` 可覆寫**物理 collision
    輪徑**（plugin 的 `wheel_radius` 保持 `params.wheel_diameter_m/2`）：
    物理輪徑與 odom 宣告輪徑分離 → 車每轉實際走 2π·r_phys，odom 記
    2π·r_declared → encoder-invisible 里程誤差（恆速打滑不存在，這是
    注入 odom 誤差的機制）。

    預設 None = 維持現有行為。
    """
    for name, value in (("left_wheel_mu", left_wheel_mu),
                        ("right_wheel_mu", right_wheel_mu),
                        ("wheel_mu2", wheel_mu2),
                        ("left_wheel_radius_m", left_wheel_radius_m),
                        ("right_wheel_radius_m", right_wheel_radius_m)):
        if value is not None and not (0.0 < value <= 1.0):
            raise ValueError(f"{name} must be in (0, 1], got {value}")
    wheel_radius_m = params.wheel_diameter_m / 2.0
    wheel_width_m = wheel_radius_m / 2.0
    caster_radius_m = wheel_radius_m / 2.0
    body_length_m, body_width_m, body_height_m = BODY_SIZE_M
    body_z_m = wheel_radius_m + body_height_m / 2.0
    rear_x_m = -params.wheel_base_m / 2.0
    caster_x_m = rear_x_m + CASTER_OFFSET_FROM_REAR_AXLE_M
    wheel_y_m = params.wheel_base_m / 2.0

    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    model = ET.SubElement(sdf, "model", {"name": model_name})
    _add_text(model, "pose", _pose(0.0, 0.0, 0.0))

    _add_box_link(
        model,
        "chassis",
        _pose(0.0, 0.0, body_z_m),
        (body_length_m, body_width_m, body_height_m),
        BODY_MASS_KG,
    )
    chassis_link = next(link for link in model.iter("link") if link.attrib.get("name") == "chassis")
    _add_camera_sensor(chassis_link, CAMERA_FRONT_X_M)
    wheel_mass_kg = BODY_MASS_KG / 12.0
    left_phys_r = wheel_radius_m if left_wheel_radius_m is None else left_wheel_radius_m
    right_phys_r = wheel_radius_m if right_wheel_radius_m is None else right_wheel_radius_m
    _add_wheel_link(
        model,
        "left_wheel",
        _pose(rear_x_m, wheel_y_m, left_phys_r),
        left_phys_r,
        left_phys_r / 2.0,
        wheel_mass_kg,
        friction_mu=left_wheel_mu,
        friction_mu2=wheel_mu2,
    )
    _add_wheel_link(
        model,
        "right_wheel",
        _pose(rear_x_m, -wheel_y_m, right_phys_r),
        right_phys_r,
        right_phys_r / 2.0,
        wheel_mass_kg,
        friction_mu=right_wheel_mu,
        friction_mu2=wheel_mu2,
    )
    for name, y_m in (
        ("front_caster_left", CASTER_Y_OFFSET_M),
        ("front_caster_right", -CASTER_Y_OFFSET_M),
    ):
        _add_caster_link(
            model,
            name,
            _pose(caster_x_m, y_m, caster_radius_m),
            caster_radius_m,
            BODY_MASS_KG / 60.0,
        )

    _add_revolute_joint(model, "left_wheel_joint", "left_wheel")
    _add_revolute_joint(model, "right_wheel_joint", "right_wheel")
    _add_fixed_joint(model, "front_caster_left_joint", "front_caster_left")
    _add_fixed_joint(model, "front_caster_right_joint", "front_caster_right")

    diff_drive = ET.SubElement(
        model,
        "plugin",
        {"filename": DIFF_DRIVE_PLUGIN_FILENAME, "name": DIFF_DRIVE_PLUGIN_NAME},
    )
    _add_text(diff_drive, "left_joint", "left_wheel_joint")
    _add_text(diff_drive, "right_joint", "right_wheel_joint")
    _add_text(diff_drive, "wheel_separation", _fmt(params.wheel_base_m))
    _add_text(diff_drive, "wheel_radius", _fmt(wheel_radius_m))
    _add_text(diff_drive, "max_linear_velocity", _fmt(max_linear_velocity_mps(params)))
    _add_text(diff_drive, "topic", "/cmd_vel_safe")
    _add_text(diff_drive, "odom_topic", "/odom")
    _add_text(diff_drive, "tf_topic", "/tf")

    odometry_publisher = ET.SubElement(
        model,
        "plugin",
        {
            "filename": ODOMETRY_PUBLISHER_PLUGIN_FILENAME,
            "name": ODOMETRY_PUBLISHER_PLUGIN_NAME,
        },
    )
    _add_text(odometry_publisher, "odom_topic", TRUE_POSE_TOPIC)
    _add_text(odometry_publisher, "odom_frame", "world")
    _add_text(odometry_publisher, "robot_base_frame", "chassis")
    _add_text(odometry_publisher, "odom_publish_frequency", "50")

    joint_state_publisher = ET.SubElement(
        model,
        "plugin",
        {"filename": JOINT_STATE_PLUGIN_FILENAME, "name": JOINT_STATE_PLUGIN_NAME},
    )
    _add_text(joint_state_publisher, "joint_name", "left_wheel_joint")
    _add_text(joint_state_publisher, "joint_name", "right_wheel_joint")

    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode")


def build_model_config(model_name: str = "vgr_diff_drive", sdf_filename: str = "model.sdf") -> str:
    """回傳 Gazebo model:// URI 解析所需的模型 manifest。"""
    model = ET.Element("model")
    _add_text(model, "name", model_name)
    _add_text(model, "version", "1.0")
    _add_text(model, "sdf", sdf_filename).set("version", SDF_VERSION)
    author = ET.SubElement(model, "author")
    _add_text(author, "name", "Vision Guided Robot")
    _add_text(model, "description", "Gazebo Fortress diff-drive model for VGR.")

    ET.indent(model, space="  ")
    return ET.tostring(model, encoding="unicode")


def write_robot_sdf(output: str = DEFAULT_OUTPUT, params: DiffDriveParams | None = None) -> None:
    """寫出模型檔、model.config 與 camera_info.json，必要時建立輸出目錄。"""
    if params is None:
        params = DiffDriveParams()
    directory = os.path.dirname(output)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        f.write(build_robot_sdf(params))
        f.write("\n")
    config_output = os.path.join(directory or ".", "model.config")
    with open(config_output, "w", encoding="utf-8") as f:
        f.write(build_model_config(sdf_filename=os.path.basename(output)))
        f.write("\n")
    camera_info_output = os.path.join(directory or ".", "camera_info.json")
    with open(camera_info_output, "w", encoding="utf-8") as f:
        json.dump(build_camera_info(), f, ensure_ascii=False, indent=2)
        f.write("\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="產生 VGR 差速車 SDF")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="輸出 model.sdf 路徑")
    args = parser.parse_args(argv)
    write_robot_sdf(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
