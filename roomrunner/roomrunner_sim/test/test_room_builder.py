"""Builds the simulated room in the course simulator (needs MuJoCo and lekiwi_sim: the conda environment;
skipped with the system python3). Run: conda activate ros_mujoco; PYTHONPATH=~/FOR26 python -m pytest roomrunner/roomrunner_sim"""
from pathlib import Path

import pytest
import yaml

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("lekiwi_sim")

from roomrunner_sim import room_builder  # noqa: E402
from roomrunner_sim.sim_config import load_room  # noqa: E402

OBJECTS = yaml.safe_load((Path(__file__).resolve().parents[2] / "roomrunner_bringup" / "config" / "objects.yaml").read_text())
MARKERS = Path.home() / "FOR26" / "ros_ws" / "src" / "aruco_localization" / "config" / "sim_room_map.yaml"


@pytest.fixture(scope="module")
def sim():
    if not MARKERS.exists():
        pytest.skip("Assignment 04 sim_room_map.yaml not found")
    room = load_room()
    room_builder.install(room, OBJECTS)
    from lekiwi_sim import LeKiwiSim
    s = LeKiwiSim(scene="aruco", aruco_markers=yaml.safe_load(MARKERS.read_text()))
    s.step(50)
    return room, s


def test_room_is_in_the_scene(sim):
    room, s = sim
    m = s.model
    for name, (x, y, _) in room["cuboids"].items():
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        assert body >= 0
        assert s.data.xpos[body] == pytest.approx([x, y, OBJECTS["colour_objects"][name]["height"] / 2], abs=0.005)
    for name in ("room_obstacle_0", "room_tray", "room_tag_20", "room_tag_40"):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "overhead") >= 0
