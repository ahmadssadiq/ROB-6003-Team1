"""The imitation-learning plans and checks (roomrunner_manipulation/il/il_tools.py), no LeRobot needed."""
import sys
from collections import Counter
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "roomrunner" / "roomrunner_manipulation" / "il"))

from il_tools import (check_episode, done_trials, episode_plan, evaluation_plan, held_out_positions,  # noqa: E402
                      load_config, log_trial, mat_marks, training_positions, yaw_arrow)

CFG = load_config()


def test_config_has_three_objects_ten_training_five_held_out():
    assert set(CFG["objects"]) == {"yellow_cuboid", "violet_hexagon", "blue_cube"}
    assert len(training_positions(CFG)) == 10 and len(held_out_positions(CFG)) == 5
    assert not set(training_positions(CFG).values()) & set(held_out_positions(CFG).values())


def test_positions_are_in_the_checked_reach():
    for x, y in CFG["positions"].values():
        assert 0.19 <= x <= 0.31 and abs(y) <= 0.12


def test_episode_plan_covers_every_training_position_twice_with_two_yaws():
    plan = episode_plan(CFG, "yellow_cuboid")
    assert len(plan) == 20
    assert Counter(pid for pid, _, _ in plan) == {pid: 2 for pid in training_positions(CFG)}
    for pid in training_positions(CFG):
        yaws = [yaw for p, _, yaw in plan if p == pid]
        assert yaws[0] != yaws[1]
    assert not any(pid.startswith("H") for pid, _, _ in plan)


def test_plans_are_repeatable_and_shuffled():
    assert episode_plan(CFG, "blue_cube") == episode_plan(CFG, "blue_cube")
    assert [p for p, _, _ in episode_plan(CFG, "blue_cube")] != sorted(p for p, _, _ in episode_plan(CFG, "blue_cube"))
    with pytest.raises(ValueError):
        episode_plan(CFG, "red_block")


def test_evaluation_plan_is_held_out_only():
    trials = evaluation_plan(CFG)
    assert len(trials) == 3 * 5 * CFG["evaluation"]["trials_per_position"]
    assert all(pid.startswith("H") for _, pid, _, _, _ in trials)
    assert len(set((o, p, t) for o, p, _, t, _ in trials)) == len(trials)


def test_log_and_resume(tmp_path):
    path = tmp_path / "pickup_trials.csv"
    log_trial(path, {"method": "act", "object": "blue_cube", "position": "H2", "trial": 1, "success": 1})
    log_trial(path, {"method": "scripted", "object": "blue_cube", "position": "H2", "trial": 1, "success": 0})
    assert done_trials(path, "act") == {("blue_cube", "H2", 1)}
    assert path.read_text().splitlines()[0] == "method,object,position,trial,yaw_deg,success,seconds,note"


def good_episode(n=300):
    return [[30 * i / n, 20, 40, 10, 0, 40 if i < n / 2 else 5] for i in range(n)]


def test_check_episode():
    assert check_episode(good_episode(), fps=30) == []
    assert any("long" in p for p in check_episode(good_episode(30), fps=30))
    still = [[0, 0, 0, 0, 0, 20]] * 300
    problems = check_episode(still, fps=30)
    assert any("barely moves" in p for p in problems) and any("gripper" in p for p in problems)
    glitch = good_episode()
    glitch[100] = [90, 20, 40, 10, 0, 40]
    assert any("jump" in p for p in check_episode(glitch, fps=30))


def test_mat_and_yaw_arrow():
    marks = {label: (u, v) for label, u, v in mat_marks(CFG)}
    assert marks["T3"] == pytest.approx((0.0, 100 * (0.19 - CFG["front_edge_x"])))
    assert marks["T5"][0] > 0                                   # +y (robot's left) is +u
    assert yaw_arrow(0) == pytest.approx((0.0, 1.0)) and yaw_arrow(90) == pytest.approx((1.0, 0.0))
