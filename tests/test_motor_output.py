import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_motor_output_mapping(tmp_path):
    test_c = tmp_path / "test_motor_output.c"
    binary = tmp_path / "test_motor_output"
    test_c.write_text(
        r'''
#include <assert.h>
#include <stdbool.h>
#include "protocol.h"
#include "motor_output.h"

int main(void) {
    vgr_motor_output_t output = vgr_motor_output_from_intent(VGR_MOTOR_STOP);
    assert(output.left_direction == VGR_MOTOR_DIR_STOP);
    assert(output.right_direction == VGR_MOTOR_DIR_STOP);
    assert(output.left_duty_percent == 0u);
    assert(output.right_duty_percent == 0u);
    assert(output.standby_enabled == false);

    output = vgr_motor_output_from_intent(VGR_MOTOR_FORWARD);
    assert(output.left_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.right_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.left_duty_percent == 25u);
    assert(output.right_duty_percent == 25u);
    assert(output.standby_enabled == true);

    output = vgr_motor_output_from_intent(VGR_MOTOR_TURN_LEFT);
    assert(output.left_direction == VGR_MOTOR_DIR_STOP);
    assert(output.right_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.left_duty_percent == 0u);
    assert(output.right_duty_percent == 25u);
    assert(output.standby_enabled == true);

    output = vgr_motor_output_from_intent(VGR_MOTOR_TURN_RIGHT);
    assert(output.left_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.right_direction == VGR_MOTOR_DIR_STOP);
    assert(output.left_duty_percent == 25u);
    assert(output.right_duty_percent == 0u);
    assert(output.standby_enabled == true);

    return 0;
}
''',
        encoding="utf-8",
    )

    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Ifirmware/common",
            str(test_c),
            "firmware/common/motor_output.c",
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_motor_output_duty_can_be_overridden_at_compile_time(tmp_path):
    test_c = tmp_path / "test_motor_output_override.c"
    binary = tmp_path / "test_motor_output_override"
    test_c.write_text(
        r'''
#include <assert.h>
#include "protocol.h"
#include "motor_output.h"

int main(void) {
    vgr_motor_output_t output = vgr_motor_output_from_intent(VGR_MOTOR_FORWARD);
    assert(output.left_duty_percent == 15u);
    assert(output.right_duty_percent == 15u);
    return 0;
}
''',
        encoding="utf-8",
    )

    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DVGR_MOTOR_BENCH_DUTY_PERCENT=15u",
            "-Ifirmware/common",
            str(test_c),
            "firmware/common/motor_output.c",
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_motor_output_left_and_right_direction_can_be_inverted(tmp_path):
    test_c = tmp_path / "test_motor_output_inverted.c"
    binary = tmp_path / "test_motor_output_inverted"
    test_c.write_text(
        r'''
#include <assert.h>
#include "protocol.h"
#include "motor_output.h"

int main(void) {
    vgr_motor_output_t output = vgr_motor_output_from_intent(VGR_MOTOR_FORWARD);
    assert(output.left_direction == VGR_MOTOR_DIR_REVERSE);
    assert(output.right_direction == VGR_MOTOR_DIR_FORWARD);

    output = vgr_motor_output_from_intent(VGR_MOTOR_TURN_RIGHT);
    assert(output.left_direction == VGR_MOTOR_DIR_REVERSE);
    assert(output.right_direction == VGR_MOTOR_DIR_STOP);

    output = vgr_motor_output_from_intent(VGR_MOTOR_STOP);
    assert(output.left_direction == VGR_MOTOR_DIR_STOP);
    assert(output.right_direction == VGR_MOTOR_DIR_STOP);

    return 0;
}
''',
        encoding="utf-8",
    )

    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DVGR_LEFT_MOTOR_INVERTED=1",
            "-Ifirmware/common",
            str(test_c),
            "firmware/common/motor_output.c",
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_motor_output_channels_can_be_swapped_at_compile_time(tmp_path):
    test_c = tmp_path / "test_motor_output_swapped.c"
    binary = tmp_path / "test_motor_output_swapped"
    test_c.write_text(
        r'''
#include <assert.h>
#include "protocol.h"
#include "motor_output.h"

int main(void) {
    vgr_motor_output_t output = vgr_motor_output_from_intent(VGR_MOTOR_TURN_LEFT);
    assert(output.left_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.right_direction == VGR_MOTOR_DIR_STOP);
    assert(output.left_duty_percent == 25u);
    assert(output.right_duty_percent == 0u);

    output = vgr_motor_output_from_intent(VGR_MOTOR_TURN_RIGHT);
    assert(output.left_direction == VGR_MOTOR_DIR_STOP);
    assert(output.right_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.left_duty_percent == 0u);
    assert(output.right_duty_percent == 25u);

    output = vgr_motor_output_from_intent(VGR_MOTOR_FORWARD);
    assert(output.left_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.right_direction == VGR_MOTOR_DIR_FORWARD);
    assert(output.left_duty_percent == 25u);
    assert(output.right_duty_percent == 25u);

    return 0;
}
''',
        encoding="utf-8",
    )

    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DVGR_SWAP_MOTOR_CHANNELS=1",
            "-Ifirmware/common",
            str(test_c),
            "firmware/common/motor_output.c",
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(binary)], check=True)
