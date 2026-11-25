"""The mission runner with fake skills: every path through the state machine, no ROS."""
import pytest

from roomrunner_planning.fsm import MissionStateMachine
from roomrunner_planning.mission import run_mission, split_legs

FETCH = [("go_to", "red_block"), ("pick", "red_block"), ("go_to", "tray"), ("place", "tray")]


class FakeSkills:
    """Skills that succeed unless told otherwise. Lists of outcomes are used up one call at a time."""

    def __init__(self, nav=None, manipulate=None, plan_ok=True, errors=None):
        self.nav = list(nav or [])
        self.manip = list(manipulate or [])
        self.plan_ok, self.errors = plan_ok, errors or []
        self.calls = []

    def refresh_map(self):
        self.calls.append("refresh")

    def check(self, steps):
        return self.errors

    def plan(self, target):
        self.calls.append(f"plan {target}")
        return self.plan_ok, "test path"

    def navigate(self, target, on_arrived):
        self.calls.append(f"navigate {target}")
        outcome = self.nav.pop(0) if self.nav else "aligned"
        if outcome == "aligned":
            on_arrived()
        return outcome

    def manipulate(self, skill, target):
        self.calls.append(f"{skill} {target}")
        return self.manip.pop(0) if self.manip else True


def states_of(steps, skills, max_retries=2):
    seen = []
    final, _ = run_mission(steps, skills, MissionStateMachine(max_retries), lambda s, d, leg: seen.append(s))
    return final, seen


def test_split_legs():
    assert split_legs(FETCH + [("go_to", "home")]) == [("red_block", "pick"), ("tray", "place"), ("home", None)]
    with pytest.raises(ValueError):
        split_legs([("pick", "red_block")])
    with pytest.raises(ValueError):
        split_legs([("go_to", "tray"), ("pick", "red_block")])


def test_fetch_follows_the_specified_state_order():
    final, seen = states_of(FETCH, FakeSkills())
    assert final == "DONE"
    assert seen == ["PARSE_TASK", "PLAN_TO_PICKUP", "NAVIGATE", "ALIGN", "PICK", "PLAN_TO_DROPOFF", "NAVIGATE",
                    "ALIGN", "PLACE", "DONE"]


def test_fetch_then_home():
    skills = FakeSkills()
    final, seen = states_of(FETCH + [("go_to", "home")], skills)
    assert final == "DONE" and seen[-4:] == ["PLAN_TO_PICKUP", "NAVIGATE", "ALIGN", "DONE"]
    assert skills.calls.count("refresh") == 5      # before each of 3 plans, after the pick and the place


def test_object_not_found():
    final, seen = states_of(FETCH, FakeSkills(errors=["'green_block' is not in the room"]))
    assert final == "FAILED" and seen == ["PARSE_TASK", "FAILED"]


def test_empty_request():
    assert states_of([], FakeSkills())[0] == "FAILED"


def test_no_valid_path():
    final, seen = states_of(FETCH, FakeSkills(plan_ok=False))
    assert final == "FAILED" and seen[-1] == "FAILED" and "NAVIGATE" not in seen


def test_pickup_failure_is_retried():
    skills = FakeSkills(manipulate=[False, True, True])
    final, seen = states_of(FETCH, skills)
    assert final == "DONE"
    assert seen.count("RETRY") == 1 and skills.calls.count("pick red_block") == 2


def test_localization_lost_then_recovered():
    final, seen = states_of(FETCH, FakeSkills(nav=["localization_lost"]))
    assert final == "DONE" and "RETRY" in seen


def test_retries_run_out():
    final, seen = states_of(FETCH, FakeSkills(manipulate=[False] * 5), max_retries=2)
    assert final == "FAILED" and seen.count("RETRY") == 3


def test_drop_off_retry_keeps_the_block():
    skills = FakeSkills(nav=["aligned", "nav_failed"])   # first leg fine, drop-off navigation fails once
    final, seen = states_of(FETCH, skills)
    assert final == "DONE"
    retry = seen.index("RETRY")
    assert seen[retry + 1] == "PLAN_TO_DROPOFF"
    assert skills.calls.count("pick red_block") == 1
