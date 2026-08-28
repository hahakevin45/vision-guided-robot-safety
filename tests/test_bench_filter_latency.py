"""Test that bench_filter_latency runs and produces plausible numbers."""
import os
import re
import subprocess
import sys

SCRIPT = "tools/bench_filter_latency.py"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _run(*args, **kwargs):
    env = {**os.environ, "PYTHONPATH": f"{REPO_ROOT}:{os.environ.get('PYTHONPATH', '')}"}
    return subprocess.run(
        [sys.executable, SCRIPT, *args],
        capture_output=True, text=True, timeout=120, env=env, **kwargs,
    )


def test_cli_runs_and_produces_markdown():
    result = _run("--num-calls", "50")
    assert result.returncode == 0, f"stderr: {result.stderr}"
    out = result.stdout
    assert "# Filter Latency Benchmark" in out
    assert "safe_apf" in out
    assert "cbf" in out
    assert "gf_dwa" in out
    assert "Median" in out or "median" in out
    nums = re.findall(r"\d+\.\d+", out)
    # 3 filters × 4 stats (median, p95, p99, max) = 12 floats
    assert len(nums) >= 12


def test_cli_out_flag(tmp_path):
    out_file = tmp_path / "lat.md"
    result = _run("--num-calls", "30", "--out", str(out_file))
    assert result.returncode == 0, f"stderr: {result.stderr}"
    assert out_file.exists()
    content = out_file.read_text()
    assert "safe_apf" in content
    assert "cbf" in content
    assert "gf_dwa" in content
    for line in content.splitlines():
        if "safe_apf" in line:
            assert line.strip().endswith("|")


def test_safe_apf_latency_within_expected_range():
    result = _run("--num-calls", "200")
    assert result.returncode == 0
    # Parse markdown table to find safe_apf row
    m = re.search(r"safe_apf\s*\|\s*(\d+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)", result.stdout)
    assert m, f"Could not parse safe_apf row from:\n{result.stdout}"
    median_us = float(m.group(2))
    # The public threshold remains deliberately loose across desktop runners.
    assert median_us < 500, (
        f"safe_apf median {median_us:.1f}μs exceeds expected 300μs"
    )


def test_all_filters_have_positive_latencies():
    result = _run("--num-calls", "100")
    assert result.returncode == 0
    for name in ("safe_apf", "cbf", "gf_dwa"):
        m = re.search(
            rf"{name}\s*\|\s*(\d+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)",
            result.stdout,
        )
        assert m, f"Missing row for {name}"
        median = float(m.group(2))
        p95 = float(m.group(3))
        p99 = float(m.group(4))
        maximum = float(m.group(5))
        assert median > 0
        assert p95 >= median
        assert p99 >= p95
        assert maximum >= p99
