"""Run the study scripts and verify they still produce the documented numbers.

These are deliberately not unit tests. This repository is a study, not a
library: each script reads top to bottom and prints its findings. What the
README promises is therefore not "these functions behave correctly" but
"clone this, install requirements.txt, run the scripts, and you get these
numbers". That promise is what is checked here.

The second purpose is regression cover for dependency drift. The versions in
requirements.txt are pinned today but will be raised eventually. If
roboticstoolbox then changes a result, the README would silently become wrong;
these tests turn that into a failure instead.

Two details worth knowing before changing this file:

* Scripts run with MPLBACKEND=Agg, so matplotlib draws into memory instead of
  opening a window. That is also how a CI runner would execute them. Two
  scripts still cannot be tested this way: fk.py and animate.py use the
  interactive 3D viewer of roboticstoolbox and block on env.hold() regardless
  of the backend. They are listed in NEEDS_DISPLAY and excluded on purpose.
* Scripts run in a temporary directory, not in the repository. plot.py writes
  movej_vs_movel.png, and a test suite must not modify the working tree it is
  testing. None of the scripts read a local file, so this is safe.

Each script is executed once and its output reused, so the whole suite takes
about a dozen seconds.
"""

import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent

# Run without a window; a CI runner has no display either.
ENV = dict(os.environ, MPLBACKEND="Agg")

# Scratch directory so that scripts writing output files leave the repo alone.
WORKDIR = tempfile.mkdtemp(prefix="ur5-tests-")

# plot.py and workspace.py only draw; they print nothing, and that is not a fault.
SILENT = {"plot.py", "workspace.py"}

HEADLESS = [
    "transforms.py", "dh.py", "ik.py", "ik_alle.py", "jacobi.py", "manip.py",
    "singular.py", "workspace.py", "traj.py", "bahnform.py", "bahn_linear.py",
    "vergleich.py", "profile.py", "singular_bahn.py", "plot.py",
]

# Interactive 3D viewer, blocks on env.hold() even with a non-interactive backend.
NEEDS_DISPLAY = ["fk.py", "animate.py"]

NUMBER = re.compile(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")

_cache: dict[str, str] = {}


def run(script):
    """Execute a script once and return its stdout; later calls reuse the result."""
    if script not in _cache:
        result = subprocess.run(
            [sys.executable, str(ROOT / script)],
            capture_output=True, text=True, timeout=600, env=ENV, cwd=WORKDIR,
        )
        assert result.returncode == 0, (
            f"{script} exited with {result.returncode}\n"
            f"--- stderr ---\n{result.stderr[-2000:]}"
        )
        _cache[script] = result.stdout
    return _cache[script]


def assert_reports(text, expected, tol, what):
    """Fail unless some number in the output lies within tol of expected.

    Matching on values rather than on formatted strings: a test that greps for
    "278.7" breaks as soon as a print statement is reformatted, which says
    nothing about the result. This one only fails when the number itself moves.
    """
    found = any(abs(float(m) - expected) <= tol for m in NUMBER.findall(text))
    assert found, (
        f"{what}: expected a value near {expected} (+/-{tol}), found none.\n"
        f"--- output ---\n{text[-1500:]}"
    )


def test_every_script_is_accounted_for():
    """No script may be silently forgotten when a new one is added."""
    on_disk = {p.name for p in ROOT.glob("*.py")}
    assert on_disk == set(HEADLESS) | set(NEEDS_DISPLAY), (
        "scripts in the repository do not match the lists in this file: "
        f"{on_disk ^ (set(HEADLESS) | set(NEEDS_DISPLAY))}"
    )


@pytest.mark.parametrize("script", HEADLESS)
def test_script_runs(script):
    """Every non-interactive script completes without an error.

    run() already fails on a non-zero exit code. The extra check here is that a
    script which is supposed to report something actually reports it: a script
    exiting 0 after printing nothing would otherwise look healthy.
    """
    out = run(script)
    if script not in SILENT:
        assert out.strip(), f"{script} exited cleanly but printed nothing"


def test_movej_bulges_off_the_straight_line():
    """README: MoveJ deviates 121 mm from the straight line."""
    assert_reports(run("bahnform.py"), 120.8, 0.5, "MoveJ deviation [mm]")


def test_movel_costs_extra_joint_travel():
    """README: 250.0 deg net against 278.7 deg actually travelled (+11.5 %)."""
    out = run("vergleich.py")
    assert_reports(out, 250.0, 0.2, "MoveJ joint travel [deg]")
    assert_reports(out, 278.7, 0.2, "MoveL joint travel [deg]")


def test_quintic_peak_velocity_factor():
    """Quintic profile: peak velocity is 1.875 * delta_q / T."""
    out = run("traj.py")
    assert_reports(out, 56.25, 0.05, "peak velocity joint 1 [deg/s]")
    assert_reports(out, 18.75, 0.05, "peak velocity joint 3 [deg/s]")


def test_trapezoidal_jerk_does_not_converge():
    """README: the trapezoidal jerk grows with the sample count, 1879 -> 7575 -> 30356."""
    out = run("profile.py")
    for n, expected in ((100, 1879.0), (400, 7575.0), (1600, 30356.0)):
        assert_reports(out, expected, expected * 0.02, f"trapezoidal jerk at n={n}")


def test_wrist_singularity_explodes():
    """README: MoveL through q5 = 0 demands 2384 deg/s, 13.2x the joint limit."""
    assert_reports(run("singular_bahn.py"), 2384.0, 25.0, "peak joint velocity [deg/s]")


def test_movel_actually_reaches_the_goal():
    """The IK chain must arrive at q1, otherwise every MoveL number is meaningless."""
    out = run("bahn_linear.py")
    for joint, angle in enumerate((-40.0, -90.0, 60.0, -60.0, 120.0), start=1):
        assert_reports(out, angle, 0.5, f"final angle of joint {joint} [deg]")


def test_plot_writes_the_result_figure():
    """plot.py must produce the figure the README shows."""
    run("plot.py")
    assert (Path(WORKDIR) / "movej_vs_movel.png").stat().st_size > 100_000
