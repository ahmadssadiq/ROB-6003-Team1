"""The offline dry run (experiments/dry_run.py): fixed request -> pickup and drop-off routes on the fake room."""
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))
from dry_run import ROOM_FILE, OfflineSkills  # noqa: E402

from roomrunner_planning.mission import run_mission
from roomrunner_planning.mock_parser import parse


def run(request):
    skills = OfflineSkills(yaml.safe_load(ROOM_FILE.read_text()), radius=0.20, standoff=0.30)
    final, _ = run_mission(parse(request), skills, request=request)
    return final, skills


def test_fixed_request_gives_pickup_and_dropoff_routes():
    final, skills = run("bring the red block to the tray")
    assert final == "DONE"
    assert [r[0] for r in skills.routes] == ["red_block", "tray"]
    # the partition at x = 1.6..1.9 m forces a detour longer than the straight line (about 2.4 m)
    assert skills.routes[0][2] > 2.5


def test_routes_stay_out_of_grown_obstacles():
    _, skills = run("bring the blue block to the tray then go home")
    from roomrunner_planning.astar import world_to_cell
    for _, points, _, _ in skills.routes:
        assert not any(skills.blocked[world_to_cell(x, y, skills.origin, skills.res)] for x, y in points)


def test_unknown_object_fails_before_driving():
    final, skills = run("bring the purple block to the tray")
    assert final == "FAILED" and skills.routes == []
