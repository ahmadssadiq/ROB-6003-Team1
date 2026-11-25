"""Keeps the Assignment 02 kinematics tests running under pytest (they have their own runner)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import assignment_02  # noqa: E402


def test_assignment_02_fk_and_ik():
    assert assignment_02.run_tests() == 0
