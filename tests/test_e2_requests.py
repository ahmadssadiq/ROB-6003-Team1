"""Checks the E2 request set itself, and the offline trial logic with fake LLMs (no network)."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))

from e2 import achieves, count_steps, load_requests, offline_trial  # noqa: E402
from roomrunner_planning import pddl  # noqa: E402

SCENE, REQUESTS = load_requests()


def test_twenty_requests_five_per_step_count():
    assert len(REQUESTS) == 20 and len({r["id"] for r in REQUESTS}) == 20
    assert sorted(r["steps"] for r in REQUESTS) == [1] * 5 + [2] * 5 + [3] * 5 + [4] * 5


@pytest.mark.parametrize("request_", REQUESTS, ids=lambda r: f"#{r['id']}")
def test_every_goal_is_reachable_with_the_stated_number_of_steps(request_):
    plan = pddl.solve(request_["goal_facts"], SCENE["objects"], SCENE["places"])
    assert achieves(plan, request_["goal_facts"])
    assert count_steps(plan) == request_["steps"]


def answering(steps_json):
    return lambda messages: steps_json


def as_json(steps):
    return json.dumps({"steps": [{"skill": s, "target": t} for s, t in steps]})


def test_offline_trial_success_and_failure():
    r6 = next(r for r in REQUESTS if r["id"] == 6)          # bring the red block to the tray
    good = [("go_to", "red_block"), ("pick", "red_block"), ("go_to", "tray"), ("place", "tray")]
    wrong = [(s, t.replace("red", "blue")) for s, t in good]
    ok = offline_trial(r6, SCENE, "direct", answering(as_json(good)))
    bad = offline_trial(r6, SCENE, "direct", answering(as_json(wrong)))
    assert (ok["valid"], ok["success"]) == (1, 1)
    assert (bad["valid"], bad["success"], bad["error"]) == (1, 0, "valid plan, wrong outcome")


def test_offline_trial_llmp():
    r19 = next(r for r in REQUESTS if r["id"] == 19)
    row = offline_trial(r19, SCENE, "llmp", answering("(:goal (and (on red_block tray) (on blue_block tray)))"))
    assert row["success"] == 1 and row["replans"] == 0


def test_mock_planner_handles_direct_wording_only():
    rows = [offline_trial(r, SCENE, "mock") for r in REQUESTS]
    direct = [row for row, r in zip(rows, REQUESTS) if r["style"] == "direct"]
    assert all(row["success"] for row in direct)
