"""The whole overhead pipeline on a synthetic picture: tags at their heights, boxes in the grid."""
import math

import numpy as np
import pytest

from roomrunner_perception.overhead_camera import OverheadCamera
from roomrunner_perception.overhead_geometry import homography_from_anchors
from roomrunner_perception.overhead_pipeline import clear_disc, make_detector, process_frame
from roomrunner_perception.synthetic import Tag, looking_down, project, render

CENTRE = (1.5, 0.0, 2.25)
OBJECTS_CFG = {"tags": {10: {"name": "red_block", "kind": "block", "height": 0.05},
                        20: {"name": "tray", "kind": "tray", "height": 0.02}},
               "robot_tag": 40, "robot_tag_height": 0.30, "robot_clear_radius": 0.30}
GRID_CFG = {"origin": [-0.5, -2.0], "width": 4.0, "height": 4.0, "resolution": 0.05}
ROBOT = Tag(40, 0.6, -0.8, z=0.30, yaw=1.2, size=0.10)
SCENE = [Tag(10, 2.4, 0.7, z=0.05, yaw=0.4), Tag(20, 0.3, 0.9, z=0.02, yaw=-0.3, size=0.08), ROBOT,
         Tag(7, 2.8, -1.0)]                                   # an id not in objects.yaml: seen, not reported
BOX = (2.0, -1.0, 2.4, -0.7)                                  # inside the view (about y = -1.2 to 1.2)
ROBOT_BODY = (0.45, -0.95, 0.75, -0.65)                       # dark base under the robot tag


@pytest.fixture(scope="module")
def setup():
    K, rvec, tvec = looking_down(CENTRE, tilt=0.03)
    camera = OverheadCamera(K=K, rvec=rvec, tvec=tvec)
    empty = render(K, rvec, tvec)
    frame = render(K, rvec, tvec, tags=SCENE, boxes=[BOX, ROBOT_BODY])
    return K, rvec, tvec, camera, empty, frame


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_objects_and_robot_found_at_their_heights(setup):
    *_, camera, empty, frame = setup
    result = process_frame(frame, camera, make_detector(), OBJECTS_CFG, GRID_CFG, empty)
    assert result["seen"] == [7, 10, 20, 40]
    found = {o["name"]: o for o in result["objects"]}
    assert set(found) == {"red_block", "tray"}
    for tag in SCENE[:2]:
        o = found[OBJECTS_CFG["tags"][tag.id]["name"]]
        assert math.hypot(o["x"] - tag.x, o["y"] - tag.y) < 0.01
        assert angle_diff(o["yaw"], tag.yaw) < math.radians(3)
    x, y, yaw = result["robot"]
    assert math.hypot(x - ROBOT.x, y - ROBOT.y) < 0.01 and angle_diff(yaw, ROBOT.yaw) < math.radians(3)


def test_floor_homography_alone_misplaces_the_robot(setup):
    """Without the camera pose (no intrinsics) the robot tag lands well off; this is what the pose fixes."""
    K, rvec, tvec, _, empty, frame = setup
    anchors = [(0.5, -0.5), (2.5, -0.5), (2.5, 1.0), (0.5, 1.0)]
    floor_only = OverheadCamera(H=homography_from_anchors(project(K, rvec, tvec, [(*a, 0) for a in anchors]),
                                                         anchors))
    x, y, _ = process_frame(frame, floor_only, make_detector(), OBJECTS_CFG, GRID_CFG, empty)["robot"]
    assert math.hypot(x - ROBOT.x, y - ROBOT.y) > 0.10


def cell(x, y):
    return int((y - GRID_CFG["origin"][1]) / 0.05), int((x - GRID_CFG["origin"][0]) / 0.05)


def test_grid_has_the_box_but_not_the_robot(setup):
    *_, camera, empty, frame = setup
    grid = process_frame(frame, camera, make_detector(), OBJECTS_CFG, GRID_CFG, empty)["grid"]
    assert grid.shape == (80, 80)
    assert grid[cell(2.2, -0.85)] == 100                # inside the box
    assert grid[cell(1.2, 0.0)] == 0                    # open floor
    assert grid[cell(0.6, -0.8)] == 0                   # under the robot: cleared
    assert grid[cell(0.6, -0.8)[0], cell(0.6, -0.8)[1] + 1] == 0


def test_no_empty_floor_picture_means_no_obstacles(setup):
    *_, camera, _, frame = setup
    grid = process_frame(frame, camera, make_detector(), OBJECTS_CFG, GRID_CFG)["grid"]
    assert (grid == 100).sum() == 0


def test_clear_disc():
    grid = np.full((10, 10), 100, np.int8)
    clear_disc(grid, origin=(0.0, 0.0), resolution=0.1, x=0.5, y=0.5, radius=0.2)
    assert grid[5, 5] == 0 and grid[4, 4] == 0 and grid[0, 0] == 100
    assert (grid == 0).sum() == 12            # cell centres within 0.2 m of (0.5, 0.5)


def test_floor_outside_the_view_is_unknown(setup):
    *_, camera, empty, frame = setup
    grid = process_frame(frame, camera, make_detector(), OBJECTS_CFG, GRID_CFG, empty)["grid"]
    assert grid[cell(1.5, -1.8)] == -1        # 1.8 m off-centre in y: outside a 2.25 m high 1080p view


def test_robot_tag_behind_the_base_centre():
    from roomrunner_perception.overhead_pipeline import base_from_tag
    # robot at (1, 2) facing +y; its tag 8 cm behind the centre, so at (1, 1.92)
    assert base_from_tag((1.0, 1.92, math.pi / 2), (-0.08, 0.0)) == pytest.approx((1.0, 2.0, math.pi / 2))
    assert base_from_tag((0.5, 0.5, 0.3), (0.0, 0.0)) == (0.5, 0.5, 0.3)
