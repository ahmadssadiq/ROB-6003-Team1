"""Colour cuboids found in synthetic overhead pictures, with the ranges of objects.yaml."""
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from roomrunner_perception.colour_objects import (colour_mask, find_colour_objects, footprint_area, foreground_mask,
                                                  polygon_area_centroid, suggest_hsv_range, symmetry)
from roomrunner_perception.overhead_camera import OverheadCamera
from roomrunner_perception.overhead_pipeline import make_detector, process_frame
from roomrunner_perception.synthetic import Cuboid, Tag, looking_down, render

CONFIG = Path(__file__).resolve().parents[2] / "roomrunner_bringup" / "config" / "objects.yaml"
OBJECTS_CFG = yaml.safe_load(CONFIG.read_text())
SPECS = OBJECTS_CFG["colour_objects"]
BGR = {"red_cuboid": (30, 30, 210), "yellow_cuboid": (0, 220, 230), "green_cuboid": (40, 180, 40),
       "blue_cuboid": (200, 90, 20), "violet_cuboid": (170, 50, 140)}
SCENE = {"red_cuboid": (0.6, 0.8, 0.2), "yellow_cuboid": (2.6, 0.9, 1.0), "green_cuboid": (1.5, 0.1, 2.0),
         "blue_cuboid": (0.4, -0.9, 2.9), "violet_cuboid": (2.8, -0.8, 0.6)}       # name: x, y, yaw


def cuboids(names=SCENE):
    return [Cuboid(*SCENE[n], colour=BGR[n]) for n in names]


@pytest.fixture(scope="module")
def camera_setup():
    K, rvec, tvec = looking_down((1.5, 0.0, 2.25), tilt=0.03)
    return K, rvec, tvec, OverheadCamera(K=K, rvec=rvec, tvec=tvec)


def yaw_error(a, b, period=math.pi):
    d = (a - b) % period
    return min(d, period - d)


def check(found, names, tolerance=0.005, min_confidence=0.8):
    by_name = {o["name"]: o for o in found}
    assert set(by_name) == set(names)
    for n in names:
        x, y, yaw = SCENE[n]
        o = by_name[n]
        assert math.hypot(o["x"] - x, o["y"] - y) < tolerance, n
        assert yaw_error(o["yaw"], yaw) < math.radians(4), n
        assert o["tag_id"] == -1 and o["confidence"] > min_confidence


def test_five_cuboids_found_everywhere_in_the_view(camera_setup):
    K, rvec, tvec, camera = camera_setup
    image = render(K, rvec, tvec, cuboids=cuboids())
    check(find_colour_objects(image, camera, SPECS), SCENE)


def test_floor_homography_only_is_within_two_cm(camera_setup):
    """Without intrinsics there is no camera pose: no height or side correction, still close enough to drive to."""
    K, rvec, tvec, camera = camera_setup
    image = render(K, rvec, tvec, cuboids=cuboids())
    floor_only = OverheadCamera(H=camera.H)
    check(find_colour_objects(image, floor_only, SPECS), SCENE, tolerance=0.02, min_confidence=0.3)


def test_with_the_empty_floor_and_only_three_cuboids(camera_setup):
    K, rvec, tvec, camera = camera_setup
    names = ["red_cuboid", "yellow_cuboid", "blue_cuboid"]
    image, empty = render(K, rvec, tvec, cuboids=cuboids(names)), render(K, rvec, tvec)
    check(find_colour_objects(image, camera, SPECS, foreground_mask(image, empty)), names)


def test_a_yellow_thing_of_the_wrong_size_is_not_the_cuboid(camera_setup):
    K, rvec, tvec, camera = camera_setup
    poster = Cuboid(1.0, -0.4, length=0.25, width=0.20, height=0.001, colour=BGR["yellow_cuboid"])
    speck = Cuboid(2.0, 0.5, length=0.012, width=0.010, height=0.001, colour=BGR["yellow_cuboid"])
    image = render(K, rvec, tvec, cuboids=[poster, speck] + cuboids(["yellow_cuboid"]))
    check(find_colour_objects(image, camera, SPECS), ["yellow_cuboid"])
    image = render(K, rvec, tvec, cuboids=[poster, speck])
    assert find_colour_objects(image, camera, SPECS) == []


def test_pipeline_reports_tags_and_cuboids_together(camera_setup):
    K, rvec, tvec, camera = camera_setup
    image = render(K, rvec, tvec, tags=[Tag(20, 2.2, -0.3, z=0.02, size=0.08)], cuboids=cuboids(["green_cuboid"]))
    grid = {"origin": [-0.5, -2.0], "width": 4.0, "height": 4.0, "resolution": 0.05}
    result = process_frame(image, camera, make_detector(), OBJECTS_CFG, grid, render(K, rvec, tvec))
    assert sorted(o["name"] for o in result["objects"]) == ["green_cuboid", "tray"]


def test_footprint_and_symmetry():
    assert footprint_area({"size": [0.064, 0.02]}) == pytest.approx(0.00128)
    assert symmetry({"size": [0.064, 0.02]}) == pytest.approx(math.pi)
    assert symmetry({"size": [0.035, 0.035]}) == pytest.approx(math.pi / 2)


def test_red_hue_range_wraps_around_zero():
    hsv = np.array([[[2, 200, 200], [175, 200, 200], [90, 200, 200]]], np.uint8)
    assert colour_mask(hsv, [170, 100, 70], [8, 255, 255]).ravel().tolist() == [255, 255, 0]


def test_polygon_area_centroid():
    area, centre = polygon_area_centroid(np.array([[0, 0], [2, 0], [2, 1], [0, 1]], float))
    assert area == pytest.approx(2.0) and centre == pytest.approx((1.0, 0.5))


def test_suggested_range_covers_the_samples():
    for bgr in BGR.values():
        hsv = cv2.cvtColor(np.full((1, 1, 3), bgr, np.uint8), cv2.COLOR_BGR2HSV).reshape(1, 3)
        samples = hsv + np.random.default_rng(1).integers(-4, 5, (50, 3))
        samples[:, 0] %= 180
        samples = np.clip(samples, 0, 255).astype(np.uint8)
        low, high = suggest_hsv_range(samples)
        mask = colour_mask(samples.reshape(1, -1, 3), low, high)
        assert mask.mean() == 255, bgr


def test_picker_overlap_and_yaml_write(tmp_path):
    from roomrunner_perception.overhead_colour_picker import hue_overlap, write_ranges
    assert hue_overlap((170, 8), (5, 20)) and not hue_overlap((20, 34), (45, 80))
    path = tmp_path / "objects.yaml"
    path.write_text(CONFIG.read_text())
    write_ranges(path, {"blue_cuboid": ([101, 90, 40], [121, 255, 255])})
    written = yaml.safe_load(path.read_text())["colour_objects"]
    assert written["blue_cuboid"]["hsv_low"] == [101, 90, 40] and written["red_cuboid"] == SPECS["red_cuboid"]
    assert "PLACEHOLDER" in path.read_text()                     # comments kept


def test_cuboids_are_marked_in_the_grid(camera_setup):
    K, rvec, tvec, camera = camera_setup
    image = render(K, rvec, tvec, cuboids=cuboids(["blue_cuboid"]))
    grid_cfg = {"origin": [-0.5, -2.0], "width": 4.0, "height": 4.0, "resolution": 0.05}
    grid = process_frame(image, camera, make_detector(), OBJECTS_CFG, grid_cfg, render(K, rvec, tvec))["grid"]
    x, y, _ = SCENE["blue_cuboid"]
    row, col = int((y + 2.0) / 0.05), int((x + 0.5) / 0.05)
    assert grid[row, col] == 100 and grid[row, col + 3] == 0


def test_coloured_parts_on_the_robot_are_ignored(camera_setup):
    K, rvec, tvec, camera = camera_setup
    on_robot = Cuboid(0.6, -0.8, colour=BGR["red_cuboid"])            # e.g. a red motor cover next to the robot tag
    image = render(K, rvec, tvec, tags=[Tag(40, 0.6, -0.8, z=0.30, size=0.10)], cuboids=[on_robot] + cuboids(["red_cuboid"]))
    grid = {"origin": [-0.5, -2.0], "width": 4.0, "height": 4.0, "resolution": 0.05}
    result = process_frame(image, camera, make_detector(), OBJECTS_CFG, grid)
    red = [o for o in result["objects"] if o["name"] == "red_cuboid"]
    assert len(red) == 1 and math.hypot(red[0]["x"] - SCENE["red_cuboid"][0], red[0]["y"] - SCENE["red_cuboid"][1]) < 0.005
