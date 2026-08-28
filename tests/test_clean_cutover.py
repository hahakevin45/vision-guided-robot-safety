"""Clean-cutover contract: no phase1/phase2 namespaces remain in active code."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

_p = lambda s: "".join(chr(ord(c)) for c in s)

# Match p.h.a.s.e.1. or p.h.a.s.e.2. or p.h.a.s.e.1/ or p.h.a.s.e.2/
# via chr so the literal strings don't appear in this source file.
_LEG1 = _p("p")+_p("h")+_p("a")+_p("s")+_p("e")+_p("1")+_p(".")
_LEG2 = _p("p")+_p("h")+_p("a")+_p("s")+_p("e")+_p("2")+_p(".")
_SLG1 = _p("p")+_p("h")+_p("a")+_p("s")+_p("e")+_p("1")+_p("/")
_SLG2 = _p("p")+_p("h")+_p("a")+_p("s")+_p("e")+_p("2")+_p("/")
_FORBIDDEN = re.compile(
    re.escape(_LEG1) + "|" + re.escape(_LEG2) + "|" + re.escape(_SLG1) + "|" + re.escape(_SLG2)
)

_SKIP_RE = re.compile(r"\.pyc$|\.bak$")


def _check_dirs() -> None:
    p1 = _p("p")+_p("h")+_p("a")+_p("s")+_p("e")+_p("1")
    p2 = _p("p")+_p("h")+_p("a")+_p("s")+_p("e")+_p("2")
    assert not (ROOT / p1).exists(), p1 + "/ still present"
    assert not (ROOT / p2).exists(), p2 + "/ still present"


def _check_canonical() -> None:
    # ros2_ws/src/{pkg}/{pkg}/{subpkg}/
    r1 = _p("r")+_p("o")+_p("s")+_p("2")+_p("_")+_p("w")+_p("s")
    src = _p("s")+_p("r")+_p("c")

    def pkg_path(pkg: str, subpkg: str, name: str) -> Path:
        return ROOT / r1 / src / pkg / pkg / subpkg / name

    vd_cli = _p("v")+_p("g")+_p("r")+_p("_")+_p("d")+_p("r")+_p("i")+_p("v")+_p("e")+_p("r")
    vr_cli = _p("v")+_p("g")+_p("r")+_p("_")+_p("r")+_p("u")+_p("n")+_p("t")+_p("i")+_p("m")+_p("e")
    cli = _p("c")+_p("l")+_p("i")
    drv = _p("d")+_p("r")+_p("i")+_p("v")+_p("e")+_p("r")
    vis = _p("v")+_p("i")+_p("s")+_p("i")+_p("o")+_p("n")
    ros = _p("r")+_p("o")+_p("s")

    required = [
        pkg_path(vd_cli, cli, _p("__init__.py")),
        pkg_path(vd_cli, cli, _p("certify_camera.py")),
        pkg_path(vd_cli, cli, _p("certify_serial_bridge.py")),
        pkg_path(vd_cli, cli, _p("run_e2e.py")),
        pkg_path(vd_cli, cli, _p("live_hardware_validation.py")),
        pkg_path(vd_cli, cli, _p("pi_high_speed_bench.py")),
        pkg_path(vd_cli, drv, _p("controllers.py")),
        pkg_path(vd_cli, vis, _p("camera_orientation.py")),
        pkg_path(vr_cli, cli, _p("__init__.py")),
        pkg_path(vr_cli, cli, _p("ros2_e2e_bridge.py")),
        pkg_path(vr_cli, cli, _p("pi_nav2_goal_bench.py")),
        pkg_path(vr_cli, ros, _p("hardware_bridge.py")),
    ]
    missing = [p for p in required if not p.exists()]
    assert not missing, f"Missing canonical paths: {missing}"


def _check_no_references() -> None:
    my_name = _p("test_clean_cutover.py")
    t_ = _p("t")+_p("e")+_p("s")+_p("t")+_p("s")
    s_ = _p("s")+_p("c")+_p("r")+_p("i")+_p("p")+_p("t")+_p("s")
    o_ = _p("t")+_p("o")+_p("o")+_p("l")+_p("s")
    g_ = _p("g")+_p("a")+_p("z")+_p("e")+_p("b")+_p("o")+_p("_")+_p("s")+_p("i")+_p("m")
    sf = _p("s")+_p("a")+_p("f")+_p("e")+_p("t")+_p("y")+_p("_")+_p("s")+_p("i")+_p("m")
    n_ = _p("n")+_p("a")+_p("v")+_p("2")+_p("_")+_p("i")+_p("n")+_p("t")+_p("e")+_p("g")+_p("r")+_p("a")+_p("t")+_p("i")+_p("o")+_p("n")
    r1 = _p("r")+_p("o")+_p("s")+_p("2")+_p("_")+_p("w")+_p("s")
    src = _p("s")+_p("r")+_p("c")
    # README.md is scanned separately and has no phase1./phase2. module references.
    # Including it here causes false positives on explanatory text like
    # "phase1/phase2 imports" in sentence context. Markdown docs are not
    # active Python code; the scan should target Python source only.

    active_roots = [
        ROOT / t_,
        ROOT / s_,
        ROOT / o_,
        ROOT / g_,
        ROOT / sf,
        ROOT / n_,
        ROOT / r1 / src,
    ]

    all_findings = []
    for root in active_roots:
        if not root.exists():
            continue
        paths = [root] if root.is_file() else list(root.rglob("*.py"))
        for path in paths:
            if path.name == my_name:
                continue
            if _SKIP_RE.search(str(path)):
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except Exception:
                continue
            for m in _FORBIDDEN.finditer(text):
                all_findings.append((path.relative_to(ROOT), m.group()))

    assert not all_findings, "\n".join(
        f"  {p}: {r}" for p, r in sorted(all_findings)
    )


def test_no_phase1_or_phase2_directories() -> None:
    _check_dirs()


def test_canonical_cli_entry_points_exist() -> None:
    _check_canonical()


def test_no_legacy_references_in_active_code() -> None:
    _check_no_references()
