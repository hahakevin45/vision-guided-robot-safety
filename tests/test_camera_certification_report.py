import json


def test_camera_certification_report_shape():
    report = {
        "camera_index": 0,
        "requested_frames": 10,
        "pass": False,
        "checks": {},
        "summary": {},
        "error": "failed to open camera index: 0",
    }

    encoded = json.dumps(report)
    decoded = json.loads(encoded)

    assert decoded["pass"] is False
    assert "checks" in decoded
    assert "summary" in decoded
    assert "error" in decoded
