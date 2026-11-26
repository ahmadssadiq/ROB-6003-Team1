"""The simulated room's camera model and the configs written for the real nodes (no MuJoCo needed)."""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from roomrunner_sim.sim_config import PLACES, floor_seen, load_room, overhead_model, write_configs
from roomrunner_perception.overhead_camera import OverheadCamera
from roomrunner_perception.overhead_camera import camera_from_config, load_intrinsics

OBJECTS = Path(__file__).resolve().parents[2] / "roomrunner_bringup" / "config" / "objects.yaml"
ROOM = load_room()


def test_camera_sees_the_grid():
    seen_x, seen_y = floor_seen(ROOM)
    assert seen_x >= ROOM["grid"]["width"] - 0.01 and seen_y >= ROOM["grid"]["height"] - 0.01


def test_camera_looks_down_with_picture_top_at_map_plus_y():
    K, rvec, tvec = overhead_model(ROOM)
    camera = OverheadCamera(K=K, rvec=rvec, tvec=tvec)
    c = ROOM["overhead_camera"]
    assert camera.to_pixel((0.0, 0.0)) == pytest.approx((c["width"] / 2, c["height"] / 2))
    u, v = camera.to_pixel((1.0, 0.5))
    assert u > c["width"] / 2 and v < c["height"] / 2          # +x to the right, +y up the picture
    assert camera.to_map(camera.to_pixel((1.2, -0.6), 0.014), 0.014) == pytest.approx((1.2, -0.6))


def test_everything_in_the_room_is_inside_the_grid():
    g = ROOM["grid"]
    inside = lambda x, y: g["origin"][0] < x < g["origin"][0] + g["width"] and g["origin"][1] < y < g["origin"][1] + g["height"]
    things = list(ROOM["cuboids"].values()) + [ROOM["tray"]["position"], ROOM["start_pose"][:2]]
    assert all(inside(p[0], p[1]) for p in things)
    known = yaml.safe_load(OBJECTS.read_text())["colour_objects"]
    assert set(ROOM["cuboids"]) <= set(known)


def test_written_configs_load_in_the_real_nodes(tmp_path):
    paths = write_configs(ROOM, OBJECTS, PLACES, tmp_path)
    overhead = yaml.safe_load(paths["overhead"].read_text())
    K, dist = load_intrinsics(overhead["intrinsics_file"])
    camera = camera_from_config(overhead, K)
    assert camera.has_pose and np.allclose(dist, 0)
    assert camera.to_map(camera.to_pixel((0.7, 0.3))) == pytest.approx((0.7, 0.3))
    floor_only = OverheadCamera(H=overhead["homography"])          # the homography agrees with the pose
    assert floor_only.to_map(camera.to_pixel((-1.0, 0.8))) == pytest.approx((-1.0, 0.8))
    objects = yaml.safe_load(paths["objects"].read_text())
    assert objects["robot_tag_offset"] == ROOM["robot_tag"]["offset"] and 20 in objects["tags"]
    assert "colour_objects" in objects and "home" in yaml.safe_load(paths["places"].read_text())["places"]
