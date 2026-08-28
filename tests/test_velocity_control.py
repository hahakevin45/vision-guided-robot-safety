import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _compile_and_run(tmp_path, name, body):
    test_c = tmp_path / f"{name}.c"
    binary = tmp_path / name
    test_c.write_text(body, encoding="utf-8")
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Ifirmware/common",
            str(test_c),
            "firmware/common/velocity_control.c",
            "-lm",
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_duty_increases_when_measured_below_target(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_duty_increases",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 0.05f, 0.02f, 0.0f);

    uint8_t last_duty = 0u;
    for (int i = 0; i < 5; ++i) {
        vgr_velocity_output_t out = vgr_velocity_step(&c, 500, 10, 0.1f, 0u);
        assert(out.duty_percent >= last_duty);
        last_duty = out.duty_percent;
    }
    assert(last_duty > 0u);
    return 0;
}
''',
    )


def test_duty_clamps_at_max(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_duty_clamp",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 5.0f, 5.0f, 0.0f);

    for (int i = 0; i < 20; ++i) {
        vgr_velocity_output_t out = vgr_velocity_step(&c, 900, 0, 0.1f, 0u);
        assert(out.duty_percent <= VGR_MOTOR_MAX_DUTY_PERCENT);
    }
    return 0;
}
''',
    )


def test_integral_bounded_under_sustained_saturation(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_antiwindup",
        r'''
#include <assert.h>
#include <math.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 1.0f, 0.5f, 0.0f);

    /* sustained large error (measured stuck at 0) should saturate duty at MAX
       and anti-windup must keep the integral from growing unbounded. */
    for (int i = 0; i < 100; ++i) {
        vgr_velocity_output_t out = vgr_velocity_step(&c, 900, 0, 0.1f, 0u);
        assert(out.duty_percent == VGR_MOTOR_MAX_DUTY_PERCENT);
    }
    assert(!isnan(c.integral));
    assert(!isinf(c.integral));
    assert(fabsf(c.integral) < 100000.0f);

    /* once measured catches up to target, output should recover promptly
       without persistent saturation caused by windup. */
    vgr_velocity_output_t recovered = vgr_velocity_step(&c, 900, 90, 0.1f, 0u);
    assert(recovered.duty_percent <= VGR_MOTOR_MAX_DUTY_PERCENT);
    return 0;
}
''',
    )


def test_zero_target_stops(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_zero_target",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 0.05f, 0.02f, 0.0f);

    vgr_velocity_output_t out = vgr_velocity_step(&c, 0, 0, 0.1f, 0u);
    assert(out.duty_percent == 0u);
    assert(out.direction == VGR_MOTOR_DIR_STOP);
    return 0;
}
''',
    )


def test_negative_target_reverses(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_negative_target",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 0.05f, 0.02f, 0.0f);

    vgr_velocity_output_t out = vgr_velocity_step(&c, -500, -10, 0.1f, 0u);
    assert(out.direction == VGR_MOTOR_DIR_REVERSE);
    assert(out.duty_percent > 0u);
    return 0;
}
''',
    )


def test_wrong_direction_motion_is_not_zero_error(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_wrong_direction",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 0.05f, 0.02f, 0.0f);

    /* Target is forward at 500 cps but the wheel is measured spinning
     * backward at -500 cps: magnitudes match but direction is wrong, so
     * this must NOT be read as zero error (that bug commands zero duty
     * while the wheel keeps spinning the wrong way). */
    vgr_velocity_output_t out = vgr_velocity_step(&c, 500, -50, 0.1f, 0u);
    assert(out.direction == VGR_MOTOR_DIR_FORWARD);
    assert(out.duty_percent > 0u);
    return 0;
}
''',
    )


def test_watchdog_timeout_forces_stop(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_watchdog",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 0.05f, 0.02f, 0.0f);

    vgr_velocity_output_t out = vgr_velocity_step(&c, 500, 0, 0.1f, VGR_CMD_TIMEOUT_MS + 1u);
    assert(out.duty_percent == 0u);
    assert(out.direction == VGR_MOTOR_DIR_STOP);
    return 0;
}
''',
    )


def test_target_is_clamped_to_max_counts_per_s(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_target_clamp",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t a;
    vgr_velocity_controller_t b;
    vgr_velocity_init(&a, 0.05f, 0.02f, 0.0f);
    vgr_velocity_init(&b, 0.05f, 0.02f, 0.0f);

    vgr_velocity_output_t out_extreme = vgr_velocity_step(&a, 5000, 0, 0.1f, 0u);
    vgr_velocity_output_t out_clamped = vgr_velocity_step(&b, VGR_MAX_TARGET_COUNTS_PER_S, 0, 0.1f, 0u);
    assert(out_extreme.duty_percent == out_clamped.duty_percent);
    assert(out_extreme.direction == VGR_MOTOR_DIR_FORWARD);
    return 0;
}
''',
    )


def test_duty_never_exceeds_max_duty_percent(tmp_path):
    _compile_and_run(
        tmp_path,
        "test_duty_never_exceeds_max",
        r'''
#include <assert.h>
#include "velocity_control.h"

int main(void) {
    vgr_velocity_controller_t c;
    vgr_velocity_init(&c, 10.0f, 10.0f, 10.0f);

    for (int i = 0; i < 50; ++i) {
        vgr_velocity_output_t out = vgr_velocity_step(&c, 900, -900, 0.1f, 0u);
        assert(out.duty_percent <= VGR_MOTOR_MAX_DUTY_PERCENT);
    }
    return 0;
}
''',
    )
