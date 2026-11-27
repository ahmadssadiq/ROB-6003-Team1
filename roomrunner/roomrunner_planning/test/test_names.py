import json

from roomrunner_planning import llm_planner
from roomrunner_planning.mock_parser import parse
from roomrunner_planning.names import resolve, resolve_goal, resolve_steps
from roomrunner_planning.validator import validate

OBJECTS = {"red_cuboid": "block", "blue_cuboid": "block", "tray": "tray"}
PLACES = {"home"}
KNOWN = set(OBJECTS) | PLACES


def test_block_words_match_the_camera_name():
    for said in ("red_block", "red_cube", "red_cuboid", "red_box", "Red_Block"):
        assert resolve(said, KNOWN) == "red_cuboid"


def test_exact_name_wins_and_unknown_stays():
    assert resolve("red_block", {"red_block", "red_cuboid"}) == "red_block"
    assert resolve("tray", KNOWN) == "tray" and resolve("home", KNOWN) == "home"
    assert resolve("purple_block", KNOWN) == "purple_block"
    assert resolve("block", KNOWN) == "block"


def test_no_guess_between_two_objects_of_one_colour():
    assert resolve("red_block", {"red_cuboid", "red_cube"}) == "red_block"


def test_mock_request_validates_against_cuboids():
    steps = resolve_steps(parse("bring the red block to the tray then go home"), KNOWN)
    assert steps[0] == ("go_to", "red_cuboid") and validate(steps, OBJECTS, PLACES) == []


def test_goal_facts():
    assert resolve_goal({("on", "red_block", "tray"), ("robot-at", "home")}, KNOWN) == \
        {("on", "red_cuboid", "tray"), ("robot-at", "home")}


def test_llm_plans_are_resolved_in_both_modes():
    direct = lambda m: json.dumps({"steps": [{"skill": "go_to", "target": "blue_cube"},
                                             {"skill": "pick", "target": "blue_cube"}]})
    steps, replans = llm_planner.plan("grab the blue cube", OBJECTS, PLACES, direct)
    assert steps == [("go_to", "blue_cuboid"), ("pick", "blue_cuboid")] and replans == 0
    llmp = lambda m: "(:goal (on red_block tray))"
    steps, _ = llm_planner.plan("red block in the tray", OBJECTS, PLACES, llmp, mode="llmp")
    assert ("pick", "red_cuboid") in steps
