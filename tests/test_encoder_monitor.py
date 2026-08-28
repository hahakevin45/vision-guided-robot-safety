from vgr_driver.cli.monitor_encoders import build_encoder_sample


def test_encoder_monitor_sample_computes_normalized_delta_and_rate():
    sample = build_encoder_sample(
        sample_index=1,
        elapsed_s=0.2,
        left_count=-120,
        right_count=55,
        prev_left_count=-100,
        prev_right_count=50,
        left_encoder_sign=-1,
        right_encoder_sign=1,
    )

    assert sample["sample_index"] == 1
    assert sample["left_raw_delta"] == -20
    assert sample["left_normalized_delta"] == 20
    assert sample["left_normalized_counts_per_s"] == 100.0
    assert sample["right_raw_delta"] == 5
    assert sample["right_normalized_delta"] == 5
    assert sample["right_normalized_counts_per_s"] == 25.0
