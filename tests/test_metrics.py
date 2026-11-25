import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))

from metrics import rmse, success_rate, task_summary  # noqa: E402


def test_rmse():
    assert rmse([(1, 0), (0, 0)], [(0, 0), (0, 0)]) == pytest.approx(math.sqrt(0.5))
    assert rmse([(3, 4)], [(0, 0)]) == pytest.approx(5.0)


def test_rmse_needs_pairs():
    with pytest.raises(ValueError):
        rmse([(0, 0)], [])


def test_success_rate():
    assert success_rate([True, False, True, True]) == 0.75


def test_task_summary():
    trials = [dict(planner="direct", valid=True, replans=0, success=True),
              dict(planner="direct", valid=False, replans=3, success=False),
              dict(planner="llmp", valid=True, replans=1, success=True)]
    s = task_summary(trials)
    assert s["direct"]["SR"] == 0.5 and s["direct"]["mean_replans"] == 1.5 and s["llmp"]["trials"] == 1
