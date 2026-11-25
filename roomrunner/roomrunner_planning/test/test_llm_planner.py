"""LLM planning with fake LLMs: nothing goes to the network."""
import json

import pytest

from roomrunner_planning import llm_planner, pddl

OBJECTS = {"red_block": "block", "blue_block": "block", "tray": "tray"}
PLACES = {"home"}
FETCH = [("go_to", "red_block"), ("pick", "red_block"), ("go_to", "tray"), ("place", "tray")]


def scripted_llm(*answers):
    """A fake LLM that gives the answers in order and records every prompt."""
    calls = []

    def llm(messages):
        calls.append(messages)
        return answers[len(calls) - 1]
    llm.calls = calls
    return llm


def as_json(steps):
    return json.dumps({"steps": [{"skill": s, "target": t} for s, t in steps]})


def test_direct_valid_first_time():
    steps, replans = llm_planner.plan("bring the red block to the tray", OBJECTS, PLACES, scripted_llm(as_json(FETCH)))
    assert steps == FETCH and replans == 0


def test_direct_replans_with_the_errors():
    bad = as_json([("pick", "red_block")])          # forgot to go there first
    llm = scripted_llm(bad, "```json\n" + as_json(FETCH) + "\n```")
    steps, replans = llm_planner.plan("bring the red block to the tray", OBJECTS, PLACES, llm)
    assert steps == FETCH and replans == 1
    assert "go_to red_block before picking" in llm.calls[1][-1]["content"]


def test_direct_gives_up():
    with pytest.raises(ValueError):
        llm_planner.plan("x", OBJECTS, PLACES, scripted_llm(*["not json"] * 4), max_replans=3)


def test_llmp_goal_is_solved_by_the_planner():
    llm = scripted_llm("(:goal (and (on red_block tray) (robot-at home)))")
    steps, replans = llm_planner.plan("put the red block on the tray and come back", OBJECTS, PLACES, llm, mode="llmp")
    assert steps == FETCH + [("go_to", "home")] and replans == 0


def test_mock_llm_end_to_end():
    llm = llm_planner.mock_llm(OBJECTS, PLACES)
    steps, _ = llm_planner.plan("bring the blue block to the tray then go home", OBJECTS, PLACES, llm)
    assert steps[-1] == ("go_to", "home")


def test_make_llm_defaults_to_mock(monkeypatch):
    monkeypatch.delenv("ROOMRUNNER_LLM_PROVIDER", raising=False)
    llm = llm_planner.make_llm(OBJECTS, PLACES)
    assert "red_block" in llm([{"role": "user", "content": "bring the red block to the tray"}])


def test_pddl_two_blocks():
    goal = pddl.parse_goal("(and (on red_block tray) (on blue_block tray))")
    steps = pddl.solve(goal, OBJECTS, PLACES)
    assert len(steps) == 8 and steps.count(("place", "tray")) == 2


def test_pddl_unknown_name():
    with pytest.raises(ValueError):
        pddl.solve(pddl.parse_goal("(on green_block tray)"), OBJECTS, PLACES)
