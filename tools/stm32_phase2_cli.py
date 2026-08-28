#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path


PROJECT_ENV = os.environ.get("STM32_PROJECT_DIR")
DEFAULT_PROJECT = Path(PROJECT_ENV).expanduser() if PROJECT_ENV else None
REPO_ROOT = Path(__file__).resolve().parents[1]
COMMON_DIR = REPO_ROOT / "firmware/common"
STM32_HAL_DIR = REPO_ROOT / "firmware/stm32_hal"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Patch, build, and flash the STM32 Phase 2 UART firmware from CLI."
    )
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT, required=DEFAULT_PROJECT is None)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--toolchain-bin",
        type=Path,
        default=find_cubeide_toolchain_bin(),
    )
    parser.add_argument("--skip-flash", action="store_true")
    parser.add_argument("--skip-patch", action="store_true")
    parser.add_argument(
        "--motor-duty-percent",
        type=int,
        default=25,
        help="Compile-time bench motor duty percentage for FORWARD/TURN commands.",
    )
    parser.add_argument(
        "--left-motor-inverted",
        action="store_true",
        help="Invert the left motor direction in compiled firmware.",
    )
    parser.add_argument(
        "--right-motor-inverted",
        action="store_true",
        help="Invert the right motor direction in compiled firmware.",
    )
    parser.add_argument(
        "--swap-motor-channels",
        action="store_true",
        help="Swap left/right motor output channels in compiled firmware.",
    )
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Flash a minimal UART smoke app that prints VGR_READY every 500 ms.",
    )
    args = parser.parse_args()

    if not args.skip_patch:
        patch_project(
            args.project,
            smoke=args.smoke,
            motor_duty_percent=args.motor_duty_percent,
            left_motor_inverted=args.left_motor_inverted,
            right_motor_inverted=args.right_motor_inverted,
            swap_motor_channels=args.swap_motor_channels,
        )
    build_project(args.project, args.toolchain_bin)
    bin_path = make_binary(args.project, args.toolchain_bin)
    if not args.skip_flash:
        flash_binary(bin_path)

    print("\nNext verification command:")
    print(
        "python3 -m vgr_driver.cli.certify_serial_bridge "
        f"--device {args.device} --baudrate {args.baudrate} "
        "--report outputs/real_mcu_serial_certification.json"
    )
    return 0


def find_cubeide_toolchain_bin() -> Path:
    candidates = sorted(
        Path("/opt/st").glob(
            "stm32cubeide_*/plugins/"
            "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*"
            "/tools/bin"
        )
    )
    if not candidates:
        return Path("/usr/bin")
    return candidates[-1]


def patch_project(
    project: Path,
    smoke: bool = False,
    motor_duty_percent: int = 25,
    left_motor_inverted: bool = False,
    right_motor_inverted: bool = False,
    swap_motor_channels: bool = False,
) -> None:
    core_src = project / "Core/Src"
    core_inc = project / "Core/Inc"
    debug_core_src = project / "Debug/Core/Src"
    main_c = core_src / "main.c"
    subdir_mk = debug_core_src / "subdir.mk"
    objects_list = project / "Debug/objects.list"

    require_file(main_c)
    require_file(subdir_mk)
    require_file(objects_list)
    core_src.mkdir(parents=True, exist_ok=True)
    core_inc.mkdir(parents=True, exist_ok=True)

    copy_phase2_sources(
        core_src,
        core_inc,
        smoke=smoke,
        motor_duty_percent=motor_duty_percent,
        left_motor_inverted=left_motor_inverted,
        right_motor_inverted=right_motor_inverted,
        swap_motor_channels=swap_motor_channels,
    )
    backup_once(main_c)
    backup_once(subdir_mk)
    backup_once(objects_list)
    patch_main_c(main_c, smoke=smoke)
    patch_subdir_mk(subdir_mk, smoke=smoke)
    patch_objects_list(objects_list, smoke=smoke)
    mode = "UART smoke" if smoke else "Phase 2"
    print(f"Patched STM32 project for {mode}: {project}")


def copy_phase2_sources(
    core_src: Path,
    core_inc: Path,
    smoke: bool = False,
    motor_duty_percent: int = 25,
    left_motor_inverted: bool = False,
    right_motor_inverted: bool = False,
    swap_motor_channels: bool = False,
) -> None:
    for name in ["protocol.c", "state_machine.c", "motor_output.c", "encoder_counter.c", "velocity_control.c"]:
        shutil.copy2(COMMON_DIR / name, core_src / name)
    for name in ["protocol.h", "state_machine.h", "motor_output.h", "encoder_counter.h", "velocity_control.h"]:
        shutil.copy2(COMMON_DIR / name, core_inc / name)
    override_motor_output_config(
        core_inc / "motor_output.h",
        duty_percent=motor_duty_percent,
        left_motor_inverted=left_motor_inverted,
        right_motor_inverted=right_motor_inverted,
        swap_motor_channels=swap_motor_channels,
    )

    app_c = (STM32_HAL_DIR / "stm32_phase2_app.c").read_text(encoding="utf-8")
    app_c = app_c.replace('#include "../common/protocol.h"', '#include "protocol.h"')
    app_c = app_c.replace('#include "../common/state_machine.h"', '#include "state_machine.h"')
    app_c = app_c.replace('#include "../common/motor_output.h"', '#include "motor_output.h"')
    app_c = app_c.replace('#include "../common/velocity_control.h"', '#include "velocity_control.h"')
    (core_src / "stm32_phase2_app.c").write_text(app_c, encoding="utf-8")
    shutil.copy2(STM32_HAL_DIR / "stm32_phase2_app.h", core_inc / "stm32_phase2_app.h")
    shutil.copy2(STM32_HAL_DIR / "stm32_motor_driver.c", core_src / "stm32_motor_driver.c")
    shutil.copy2(STM32_HAL_DIR / "stm32_motor_driver.h", core_inc / "stm32_motor_driver.h")
    shutil.copy2(STM32_HAL_DIR / "stm32_encoder.c", core_src / "stm32_encoder.c")
    shutil.copy2(STM32_HAL_DIR / "stm32_encoder.h", core_inc / "stm32_encoder.h")
    if smoke:
        shutil.copy2(STM32_HAL_DIR / "stm32_uart_smoke_app.c", core_src / "stm32_uart_smoke_app.c")
        shutil.copy2(STM32_HAL_DIR / "stm32_uart_smoke_app.h", core_inc / "stm32_uart_smoke_app.h")


def patch_main_c(path: Path, smoke: bool = False) -> None:
    text = path.read_text(encoding="utf-8")
    include = '#include "stm32_uart_smoke_app.h"' if smoke else '#include "stm32_phase2_app.h"'
    init = "stm32_uart_smoke_app_init(&huart2);" if smoke else "stm32_phase2_app_init(&huart2);"
    poll = "stm32_uart_smoke_app_poll();" if smoke else "stm32_phase2_app_poll();"

    text = remove_app_lines(text)
    text = insert_once(
        text,
        marker="/* USER CODE BEGIN Includes */",
        addition=f"/* USER CODE BEGIN Includes */\n{include}",
    )
    text = insert_once(
        text,
        marker="/* USER CODE BEGIN 2 */",
        addition=f"/* USER CODE BEGIN 2 */\n  {init}",
    )

    old_loop = "\t  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);\n\t  HAL_Delay(500);\n"
    if old_loop in text:
        text = text.replace(old_loop, f"    {poll}\n", 1)
    elif "stm32_phase2_app_poll();" not in text and "stm32_uart_smoke_app_poll();" not in text:
        text = text.replace(
            "  while (1)\n  {",
            f"  while (1)\n  {{\n    {poll}",
            1,
        )
    path.write_text(text, encoding="utf-8")


def override_motor_output_config(
    path: Path,
    duty_percent: int,
    left_motor_inverted: bool,
    right_motor_inverted: bool,
    swap_motor_channels: bool = False,
) -> None:
    if not 0 <= duty_percent <= 100:
        raise ValueError("motor duty percent must be between 0 and 100")
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "#define VGR_MOTOR_BENCH_DUTY_PERCENT 25u",
        f"#define VGR_MOTOR_BENCH_DUTY_PERCENT {duty_percent}u",
    )
    text = text.replace(
        "#define VGR_LEFT_MOTOR_INVERTED 0",
        f"#define VGR_LEFT_MOTOR_INVERTED {1 if left_motor_inverted else 0}",
    )
    text = text.replace(
        "#define VGR_RIGHT_MOTOR_INVERTED 0",
        f"#define VGR_RIGHT_MOTOR_INVERTED {1 if right_motor_inverted else 0}",
    )
    text = text.replace(
        "#define VGR_SWAP_MOTOR_CHANNELS 0",
        f"#define VGR_SWAP_MOTOR_CHANNELS {1 if swap_motor_channels else 0}",
    )
    path.write_text(text, encoding="utf-8")


def patch_subdir_mk(path: Path, smoke: bool = False) -> None:
    text = path.read_text(encoding="utf-8")
    names = [
        "protocol",
        "state_machine",
        "motor_output",
        "encoder_counter",
        "velocity_control",
        "stm32_phase2_app",
        "stm32_motor_driver",
        "stm32_encoder",
    ]
    if smoke:
        names.append("stm32_uart_smoke_app")
    text = rewrite_make_list(
        text,
        "C_SRCS += \\",
        [f"../Core/Src/{name}.c" for name in names],
    )
    text = rewrite_make_list(
        text,
        "OBJS += \\",
        [f"./Core/Src/{name}.o" for name in names],
    )
    text = rewrite_make_list(
        text,
        "C_DEPS += \\",
        [f"./Core/Src/{name}.d" for name in names],
    )
    path.write_text(text, encoding="utf-8")


def patch_objects_list(path: Path, smoke: bool = False) -> None:
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    names = [
        "protocol",
        "state_machine",
        "motor_output",
        "encoder_counter",
        "velocity_control",
        "stm32_phase2_app",
        "stm32_motor_driver",
        "stm32_encoder",
    ]
    if smoke:
        names.append("stm32_uart_smoke_app")
    for name in names:
        entry = f'"./Core/Src/{name}.o"'
        if entry not in lines:
            lines.insert(1, entry)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def remove_app_lines(text: str) -> str:
    blocked = (
        '#include "stm32_phase2_app.h"',
        '#include "stm32_uart_smoke_app.h"',
        "stm32_phase2_app_init(&huart2);",
        "stm32_uart_smoke_app_init(&huart2);",
        "stm32_phase2_app_poll();",
        "stm32_uart_smoke_app_poll();",
    )
    kept = []
    for line in text.splitlines():
        if any(token in line for token in blocked):
            continue
        kept.append(line)
    return "\n".join(kept) + "\n"


def build_project(project: Path, toolchain_bin: Path) -> None:
    make_path = project / "Debug"
    require_file(make_path / "makefile")
    env_path = f"{toolchain_bin}:{Path('/usr/bin')}"
    print(f"Building with PATH={env_path}")
    env = os.environ.copy()
    env["PATH"] = env_path
    subprocess.run(
        ["make", "-C", str(make_path), "all", "-j"],
        check=True,
        env=env,
    )


def make_binary(project: Path, toolchain_bin: Path) -> Path:
    elf = project / "Debug/blink_led_try.elf"
    bin_path = project / "Debug/blink_led_try.bin"
    require_file(elf)
    objcopy = toolchain_bin / "arm-none-eabi-objcopy"
    subprocess.run([str(objcopy), "-O", "binary", str(elf), str(bin_path)], check=True)
    print(f"Generated binary: {bin_path}")
    return bin_path


def flash_binary(bin_path: Path) -> None:
    require_file(bin_path)
    subprocess.run(["st-flash", "--reset", "write", str(bin_path), "0x8000000"], check=True)
    print("Flashed STM32 firmware and requested reset.")


def insert_once(text: str, marker: str, addition: str) -> str:
    inserted_line = addition.splitlines()[-1]
    if inserted_line in text:
        return text
    if marker not in text:
        raise RuntimeError(f"marker not found in main.c: {marker}")
    return text.replace(marker, addition, 1)


def rewrite_make_list(text: str, section_start: str, additions: list[str]) -> str:
    index = text.find(section_start)
    if index < 0:
        raise RuntimeError(f"section not found in subdir.mk: {section_start}")
    section_end = text.find("\n\n", index)
    if section_end < 0:
        raise RuntimeError(f"section end not found in subdir.mk: {section_start}")

    section = text[index:section_end]
    entries: list[str] = []
    for line in section.splitlines()[1:]:
        item = line.strip().rstrip("\\").strip()
        if item:
            entries.append(item)
    for addition in additions:
        if addition not in entries:
            entries.append(addition)

    rebuilt = [section_start]
    for offset, entry in enumerate(entries):
        suffix = " \\" if offset < len(entries) - 1 else ""
        rebuilt.append(f"{entry}{suffix}")
    return text[:index] + "\n".join(rebuilt) + text[section_end:]


def backup_once(path: Path) -> None:
    backup = path.with_suffix(path.suffix + ".vgr_driver.bak")
    if not backup.exists():
        shutil.copy2(path, backup)


def require_file(path: Path) -> None:
    if not path.exists():
        raise FileNotFoundError(path)


if __name__ == "__main__":
    raise SystemExit(main())
