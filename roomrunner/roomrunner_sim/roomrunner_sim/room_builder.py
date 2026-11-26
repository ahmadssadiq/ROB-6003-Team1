"""Adds the RoomRunner room to the course simulator's ArUco scene, without changing the course code.

The course LeKiwiSim builds its "aruco" scene by calling lekiwi_sim.simulator._add_aruco_markers(spec,
marker_map) while the model is still a MjSpec (before it is compiled). install(room, objects) replaces that
function, in this process only, by one that adds the wall markers as before and then our room:

    obstacles     static boxes on the floor (sim_room.yaml obstacles)
    cuboids       free (pushable, graspable) boxes in the colours of objects.yaml colour_objects
    tray          a low box with its tag (id 20) on top
    robot tag     tag 40 on a card on the robot's base, offset as in sim_room.yaml robot_tag
    overhead      a camera looking straight down over the arena (sim_room.yaml overhead_camera)

Frames: MuJoCo's world = the map frame. The robot model's base body has its front along +y and its left
along -x (lekiwi_sim); our base frame has x forward, y left, so base (x, y) is model (-y, x).
"""
import math

import mujoco
import numpy as np

from roomrunner_perception.colour_objects import middle_colour
from roomrunner_perception.overhead_pipeline import marker_image

CARD_THICKNESS = 0.002
MARKER_PIXELS = 192
BORDER = 0.25                   # white border around the printed marker, as a fraction of its size


def tag_material(spec, cv2_dictionary, tag_id):
    """A material with the printed tag (white border included) as its texture."""
    border = round(MARKER_PIXELS * BORDER)
    picture = np.full((MARKER_PIXELS + 2 * border,) * 2, 255, np.uint8)
    picture[border:border + MARKER_PIXELS, border:border + MARKER_PIXELS] = marker_image(cv2_dictionary, tag_id,
                                                                                         MARKER_PIXELS)
    texture = spec.add_texture(name=f"room_tag_{tag_id}_texture", type=mujoco.mjtTexture.mjTEXTURE_2D,
                               width=picture.shape[1], height=picture.shape[0], nchannel=3)
    texture.data = np.repeat(picture[:, :, None], 3, axis=2).tobytes()
    material = spec.add_material(name=f"room_tag_{tag_id}_material")
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = texture.name
    return material.name


def add_tag_card(spec, body, cv2_dictionary, tag_id, size, position, yaw):
    """A flat card facing up with a tag printed on it, its top at `position`; the tag's x axis at `yaw`
    (rad, in the frame of `body`)."""
    card = size * (1 + 2 * BORDER)
    body.add_geom(name=f"room_tag_{tag_id}", type=mujoco.mjtGeom.mjGEOM_BOX,
                  pos=[position[0], position[1], position[2] - CARD_THICKNESS / 2],
                  quat=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
                  size=[card / 2, card / 2, CARD_THICKNESS / 2], material=tag_material(spec, cv2_dictionary, tag_id),
                  contype=0, conaffinity=0)


def add_room(spec, room, objects, base_body):
    import cv2
    from lekiwi_sim.simulator import BIT_ARM_PROXY, BIT_JAW, BIT_WORLD   # the course model's collision bits
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    world = spec.worldbody
    box = mujoco.mjtGeom.mjGEOM_BOX

    # A plain grey floor instead of the blue checkerboard (closer to a real floor, and not cuboid-coloured).
    # Dark grey, because the course scene's lights add up to more than full brightness on a lighter one.
    # Matte (no specular highlight), or the light above makes a bright spot under the overhead camera.
    matte = spec.add_material(name="room_floor", rgba=[0.35, 0.35, 0.34, 1.0], specular=0.0, shininess=0.0,
                              reflectance=0.0)
    floor = spec.geom("floor")
    floor.material = matte.name

    for i, (x0, y0, x1, y1, height) in enumerate(room["obstacles"]):
        world.add_geom(name=f"room_obstacle_{i}", type=box, pos=[(x0 + x1) / 2, (y0 + y1) / 2, height / 2],
                       size=[(x1 - x0) / 2, (y1 - y0) / 2, height / 2], rgba=[0.45, 0.40, 0.35, 1.0],
                       contype=BIT_WORLD, conaffinity=BIT_WORLD | BIT_ARM_PROXY | BIT_JAW)

    for name, (x, y, yaw) in room["cuboids"].items():
        spec_ = objects["colour_objects"][name]
        b, g, r = middle_colour(spec_)
        length, width = spec_["size"]
        h = spec_["height"]
        body = world.add_body(name=name, pos=[x, y, h / 2], quat=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
        body.add_freejoint(name=f"{name}_free")
        body.add_geom(name=f"{name}_geom", type=box, size=[length / 2, width / 2, h / 2], mass=0.02,
                      rgba=[r / 255, g / 255, b / 255, 1.0], friction=[2.0, 0.05, 0.001], condim=4,
                      contype=BIT_WORLD, conaffinity=BIT_WORLD | BIT_JAW)   # as the course's own cubes

    tray = room["tray"]
    (tx, ty), (sx, sy, sz) = tray["position"], tray["size"]
    world.add_geom(name="room_tray", type=box, pos=[tx, ty, sz / 2], size=[sx / 2, sy / 2, sz / 2],
                   quat=[math.cos(tray["yaw"] / 2), 0.0, 0.0, math.sin(tray["yaw"] / 2)], rgba=[0.9, 0.9, 0.9, 1.0],
                   contype=BIT_WORLD, conaffinity=BIT_WORLD | BIT_ARM_PROXY | BIT_JAW)
    add_tag_card(spec, world, dictionary, tray["tag_id"], tray["tag_size"], (tx, ty, sz + 0.002), tray["yaw"])

    tag = room["robot_tag"]
    dx, dy = tag["offset"]
    base_height = 0.035                     # the base body's origin above the floor (lekiwi_sim)
    # tag x axis along the robot's front: model +y, a quarter turn from model +x
    add_tag_card(spec, base_body, dictionary, tag["id"], tag["size"], (-dy, dx, tag["height"] - base_height),
                 math.pi / 2)

    c = room["overhead_camera"]
    world.add_camera(name="overhead", pos=list(c["position"]), quat=[1.0, 0.0, 0.0, 0.0], fovy=c["fovy_deg"])


def install(room, objects):
    """Make the course simulator's ArUco scene include our room (in this process)."""
    from lekiwi_sim import simulator
    original = simulator._add_aruco_markers

    def markers_and_room(spec, marker_map):
        original(spec, marker_map)
        add_room(spec, room, objects, spec.body(simulator.MODEL_BASE_BODY))

    simulator._add_aruco_markers = markers_and_room
