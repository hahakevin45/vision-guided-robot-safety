import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_encoder_counter_tracks_quadrature_edges_and_invalid_transitions(tmp_path):
    test_c = tmp_path / "test_encoder_counter.c"
    binary = tmp_path / "test_encoder_counter"
    test_c.write_text(
        r'''
#include <assert.h>
#include "encoder_counter.h"

int main(void) {
    vgr_encoder_counter_t counter;
    vgr_encoder_counter_init(&counter, 0u);

    assert(vgr_encoder_counter_update(&counter, 1u) == -1);
    assert(vgr_encoder_counter_update(&counter, 3u) == -1);
    assert(vgr_encoder_counter_update(&counter, 2u) == -1);
    assert(vgr_encoder_counter_update(&counter, 0u) == -1);
    assert(counter.count == -4);
    assert(counter.flags == 0u);

    assert(vgr_encoder_counter_update(&counter, 3u) == 0);
    assert(counter.count == -4);
    assert((counter.flags & VGR_ENCODER_FLAG_INVALID_TRANSITION) != 0u);
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
            "firmware/common/encoder_counter.c",
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_encoder_counter_counts_a_channel_edges_without_exti_line_conflict(tmp_path):
    test_c = tmp_path / "test_encoder_a_edge.c"
    binary = tmp_path / "test_encoder_a_edge"
    test_c.write_text(
        r'''
#include <assert.h>
#include "encoder_counter.h"

int main(void) {
    vgr_encoder_counter_t counter;
    vgr_encoder_counter_init(&counter, 0u);

    assert(vgr_encoder_counter_update_a_edge(&counter, 2u) == 1);
    assert(vgr_encoder_counter_update_a_edge(&counter, 1u) == 1);
    assert(counter.count == 2);

    assert(vgr_encoder_counter_update_a_edge(&counter, 2u) == 1);
    assert(vgr_encoder_counter_update_a_edge(&counter, 1u) == 1);
    assert(counter.count == 4);
    assert(counter.flags == 0u);
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
            "firmware/common/encoder_counter.c",
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(binary)], check=True)
