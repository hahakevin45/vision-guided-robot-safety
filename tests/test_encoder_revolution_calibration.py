from vgr_driver.cli.calibrate_encoder_revolution import build_revolution_calibration


def test_revolution_calibration_normalizes_forward_counts():
    report = build_revolution_calibration(
        wheel="left",
        left_before=-100,
        right_before=50,
        left_after=-520,
        right_after=54,
        left_encoder_sign=-1,
        right_encoder_sign=1,
    )

    assert report["wheel"] == "left"
    assert report["left"]["raw_delta"] == -420
    assert report["left"]["normalized_delta"] == 420
    assert report["left"]["counts_per_rev"] == 420
    assert report["right"]["raw_delta"] == 4
    assert report["odom_recommendation"]["left_counts_per_rev"] == 420
    assert report["odom_recommendation"]["right_counts_per_rev"] is None


def test_revolution_calibration_supports_both_wheels():
    report = build_revolution_calibration(
        wheel="both",
        left_before=-10,
        right_before=20,
        left_after=-330,
        right_after=360,
        left_encoder_sign=-1,
        right_encoder_sign=1,
    )

    assert report["left"]["counts_per_rev"] == 320
    assert report["right"]["counts_per_rev"] == 340
    assert report["odom_recommendation"]["left_counts_per_rev"] == 320
    assert report["odom_recommendation"]["right_counts_per_rev"] == 340


def test_revolution_calibration_averages_multiple_revolutions():
    report = build_revolution_calibration(
        wheel="left",
        left_before=-100,
        right_before=50,
        left_after=-4450,
        right_after=50,
        left_encoder_sign=-1,
        right_encoder_sign=1,
        revolutions=3,
    )

    assert report["revolutions"] == 3
    assert report["left"]["total_counts"] == 4350
    assert report["left"]["counts_per_rev"] == 1450.0
    assert report["odom_recommendation"]["left_counts_per_rev"] == 1450.0
