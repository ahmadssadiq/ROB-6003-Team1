import numpy as np

from roomrunner_planning.reachability import Grid
from roomrunner_planning.validator import validate


def walled_room():
    """4 x 2 m room at 0.1 m cells, a full-height wall at x = 2.0..2.2 m splitting it in two."""
    grid = np.zeros((20, 40), dtype=np.int8)
    grid[:, 20:22] = 100
    return grid


def test_from_occupancy_blocks_occupied_and_unknown():
    data = np.zeros((10, 10), dtype=np.int8)
    data[0, 0], data[9, 9] = 100, -1
    grid = Grid.from_occupancy(data.flatten().tolist(), 10, 10, 0.1, (0.0, 0.0), robot_radius=0.0)
    assert grid.blocked[0, 0] and grid.blocked[9, 9] and not grid.blocked[5, 5]


def test_same_side_reachable_other_side_not():
    g = walled_room()
    grid = Grid.from_occupancy(g.flatten().tolist(), 40, 20, 0.1, (0.0, 0.0), robot_radius=0.2)
    assert grid.reachable((0.5, 1.0), (1.5, 0.5))
    assert not grid.reachable((0.5, 1.0), (3.5, 1.0))


def test_object_inside_its_grown_obstacle_is_still_reachable():
    g = np.zeros((20, 40), dtype=np.int8)
    g[10, 30] = 100                                   # the block itself shows up in the grid
    grid = Grid.from_occupancy(g.flatten().tolist(), 40, 20, 0.1, (0.0, 0.0), robot_radius=0.2)
    assert grid.reachable((0.5, 1.0), (3.05, 1.05))


def test_validator_reports_unreachable_target():
    g = walled_room()
    grid = Grid.from_occupancy(g.flatten().tolist(), 40, 20, 0.1, (0.0, 0.0), robot_radius=0.2)
    reachable = grid.reachable_function((0.5, 1.0), {"red_cuboid": (3.5, 1.0), "tray": (1.0, 0.5)})
    objects = {"red_cuboid": "block", "tray": "tray"}
    steps = [("go_to", "red_cuboid"), ("pick", "red_cuboid"), ("go_to", "tray"), ("place", "tray")]
    errors = validate(steps, objects, {"home"}, reachable)
    assert any("no path to red_cuboid" in e for e in errors)
    assert reachable("tray") and reachable("home")    # home has no position given: not checked
