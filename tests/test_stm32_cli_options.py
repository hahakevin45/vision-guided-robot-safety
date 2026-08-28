import importlib

from pathlib import Path

from tools import stm32_phase2_cli


def test_missing_project_env_requires_explicit_path(monkeypatch):
    monkeypatch.delenv("STM32_PROJECT_DIR", raising=False)
    module = importlib.reload(stm32_phase2_cli)
    assert module.DEFAULT_PROJECT is None


def test_motor_output_header_overrides_duty_and_inversion(tmp_path):
    header = tmp_path / "motor_output.h"
    header.write_text(
        "\n".join(
            [
                "#ifndef VGR_MOTOR_BENCH_DUTY_PERCENT",
                "#define VGR_MOTOR_BENCH_DUTY_PERCENT 25u",
                "#endif",
                "#ifndef VGR_LEFT_MOTOR_INVERTED",
                "#define VGR_LEFT_MOTOR_INVERTED 0",
                "#endif",
                "#ifndef VGR_RIGHT_MOTOR_INVERTED",
                "#define VGR_RIGHT_MOTOR_INVERTED 0",
                "#endif",
                "#ifndef VGR_SWAP_MOTOR_CHANNELS",
                "#define VGR_SWAP_MOTOR_CHANNELS 0",
                "#endif",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    stm32_phase2_cli.override_motor_output_config(
        header,
        duty_percent=15,
        left_motor_inverted=True,
        right_motor_inverted=False,
        swap_motor_channels=True,
    )

    text = header.read_text(encoding="utf-8")
    assert "#define VGR_MOTOR_BENCH_DUTY_PERCENT 15u" in text
    assert "#define VGR_LEFT_MOTOR_INVERTED 1" in text
    assert "#define VGR_RIGHT_MOTOR_INVERTED 0" in text
    assert "#define VGR_SWAP_MOTOR_CHANNELS 1" in text
