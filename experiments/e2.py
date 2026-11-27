"""E2 request set: loading, step counting and offline planning trials (no ROS needed).

    requests = load_requests()                 # experiments/e2_requests.yaml
    row = offline_trial(request, scene, "direct", llm)
"""
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "roomrunner" / "roomrunner_planning"))

from roomrunner_planning import llm_planner, pddl  # noqa: E402
from roomrunner_planning.goal_check import achieves  # noqa: E402
from roomrunner_planning.mission import split_legs  # noqa: E402
from roomrunner_planning.mock_parser import parse as mock_parse  # noqa: E402
from roomrunner_planning.names import resolve_goal, resolve_steps  # noqa: E402
from roomrunner_planning.validator import validate  # noqa: E402

REQUESTS_FILE = Path(__file__).resolve().parent / "e2_requests.yaml"
COLUMNS = ["id", "text", "steps", "category", "style", "planner", "valid", "replans", "success", "plan", "error",
           "seconds"]


def load_requests(path=REQUESTS_FILE):
    """Returns (scene, requests). scene = {"objects": {name: kind}, "places": set}; every request gets its goal
    parsed into facts under "goal_facts", with its names matched to the scene ("red_block" -> "red_cuboid")."""
    data = yaml.safe_load(Path(path).read_text())
    scene = {"objects": dict(data["scene"]["objects"]), "places": set(data["scene"]["places"])}
    requests = data["requests"]
    for r in requests:
        r["goal_facts"] = resolve_goal(pddl.parse_goal(r["goal"]), set(scene["objects"]) | scene["places"])
    return scene, requests


def count_steps(plan):
    """Number of task steps in a plan: one per leg (a go_to with its pick or place, or a go_to alone)."""
    return len(split_legs(plan))


def plan_text(plan):
    return ", ".join(f"{skill}({target})" for skill, target in plan)


def offline_trial(request, scene, planner, llm=None, max_replans=3):
    """Plan one request with one planner (mock, direct or llmp) and judge it. Returns a CSV row (dict).
    valid: a plan passed the validator; success: valid and it reaches exactly the request's goal."""
    row = {k: request.get(k, "") for k in ("id", "text", "steps", "category", "style")}
    row.update(planner=planner, valid=0, replans=0, success=0, plan="", error="")
    start = time.monotonic()
    try:
        if planner == "mock":
            plan = resolve_steps(mock_parse(request["text"]), set(scene["objects"]) | scene["places"])
            errors = validate(plan, scene["objects"], scene["places"])
            if errors:
                raise ValueError("; ".join(errors))
            replans = 0
        else:
            plan, replans = llm_planner.plan(request["text"], scene["objects"], scene["places"], llm, mode=planner,
                                             max_replans=max_replans)
        row.update(valid=1, replans=replans, plan=plan_text(plan), success=int(achieves(plan, request["goal_facts"])))
        if not row["success"]:
            row["error"] = "valid plan, wrong outcome"
    except Exception as error:      # parsing, validation, re-planning exhausted, network
        row["error"] = str(error)[:300]
        if planner != "mock" and "after re-planning" in str(error):
            row["replans"] = max_replans
    row["seconds"] = round(time.monotonic() - start, 2)
    return row
