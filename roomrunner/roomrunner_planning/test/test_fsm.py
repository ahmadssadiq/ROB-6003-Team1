import pytest

from roomrunner_planning.fsm import MissionStateMachine


def run(m, *events):
    return [m.handle(e) if isinstance(e, str) else m.handle(e[0], **e[1]) for e in events]


def test_full_mission_happy_path():
    m = MissionStateMachine()
    states = run(m, "request", "parsed", "path_found", "arrived", "aligned", ("picked", {"holding": "red_block", "more_steps": True}),
                 "path_found", "arrived", "aligned", "placed")
    assert states == ["PARSE_TASK", "PLAN_TO_PICKUP", "NAVIGATE", "ALIGN", "PICK", "PLAN_TO_DROPOFF",
                      "NAVIGATE", "ALIGN", "PLACE", "DONE"]
    assert m.holding is None


def test_object_not_found_fails():
    m = MissionStateMachine()
    assert run(m, "request", "parse_failed") == ["PARSE_TASK", "FAILED"]
    assert m.handle("reset") == "IDLE"


def test_no_path_fails():
    m = MissionStateMachine()
    assert run(m, "request", "parsed", "no_path")[-1] == "FAILED"


def test_pick_failure_retries_then_succeeds():
    m = MissionStateMachine(max_retries=2)
    run(m, "request", "parsed", "path_found", "arrived", "aligned")
    assert m.handle("pick_failed") == "RETRY"
    assert m.handle("retry") == "PLAN_TO_PICKUP"
    assert m.retries == 1


def test_retries_run_out():
    m = MissionStateMachine(max_retries=1)
    run(m, "request", "parsed", "path_found")
    assert m.handle("localization_lost") == "RETRY"
    assert m.handle("retry") == "PLAN_TO_PICKUP"
    run(m, "path_found")
    assert m.handle("nav_failed") == "RETRY"
    assert m.handle("retry") == "FAILED"


def test_retry_while_holding_goes_back_to_dropoff():
    m = MissionStateMachine()
    run(m, "request", "parsed", "path_found", "arrived", "aligned", ("picked", {"holding": "red_block", "more_steps": True}), "path_found")
    assert m.handle("nav_failed") == "RETRY"
    assert m.handle("retry") == "PLAN_TO_DROPOFF"


def test_invalid_event_raises():
    m = MissionStateMachine()
    with pytest.raises(ValueError):
        m.handle("picked")


def test_more_steps_after_place_continue():
    m = MissionStateMachine()
    run(m, "request", "parsed", "path_found", "arrived", "aligned", ("picked", {"holding": "red_block", "more_steps": True}),
        "path_found", "arrived", "aligned")
    assert m.handle("placed", more_steps=True) == "PLAN_TO_PICKUP"     # then "go home"
    run(m, "path_found", "arrived")
    assert m.handle("goal_reached") == "DONE"


def test_lone_pick_is_done():
    m = MissionStateMachine()
    run(m, "request", "parsed", "path_found", "arrived", "aligned")
    assert m.handle("picked", holding="blue_block") == "DONE" and m.holding == "blue_block"
