"""Overhead camera node (runs on Mourya's laptop): one frame every capture_period seconds, or on request.

Publishes (latched):
    occupancy_grid  nav_msgs/OccupancyGrid in the map frame (0 free, 100 occupied, -1 not seen)
    objects         roomrunner_interfaces/ObjectPoseArray in the map frame: tagged blocks and tray
    overhead_robot  geometry_msgs/PoseStamped in the map frame: the robot-top tag, if seen (E1 ground truth)
Service:
    refresh_map     std_srvs/Trigger: take a frame now (the mission FSM calls it before planning and after
                    each pick or place)
Parameters:
    camera_file     overhead_camera.yaml (calibration, grid)      objects_file   objects.yaml (tag table)

The camera is opened only for each capture, so it is free in between and the laptop does little work.
The image processing itself is overhead_pipeline.process_frame (plain Python, tested on synthetic pictures).
"""
import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from roomrunner_interfaces.msg import ObjectPose, ObjectPoseArray
from roomrunner_perception.overhead_camera import OverheadCamera
from roomrunner_perception.overhead_pipeline import make_detector, process_frame, yaw_to_quaternion_zw

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_intrinsics(path):
    """camera_matrix and distortion from an Assignment 04 style calibration file, or (None, None)."""
    if not path:
        return None, None
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], float).reshape(3, 3)
    return K, np.array(data["distortion_coefficients"]["data"], float)


def camera_from_config(cfg, K):
    """OverheadCamera from overhead_camera.yaml: the camera pose when calibration found one (needs the
    intrinsics), otherwise the floor homography."""
    pose = cfg.get("camera_pose") or {}
    if K is not None and pose.get("rvec"):
        return OverheadCamera(K=K, rvec=pose["rvec"], tvec=pose["tvec"])
    return OverheadCamera(H=np.array(cfg["homography"], float))


class OverheadNode(Node):
    def __init__(self):
        super().__init__("overhead_node")
        self.declare_parameter("camera_file", "")
        self.declare_parameter("objects_file", "")
        with open(self.get_parameter("camera_file").value) as f:
            self.cfg = yaml.safe_load(f)
        with open(self.get_parameter("objects_file").value) as f:
            self.objects_cfg = yaml.safe_load(f)
        if not self.cfg.get("calibrated") or not self.cfg.get("homography"):
            raise RuntimeError("The overhead camera is not calibrated yet: run calibrate_overhead first.")
        self.K, self.dist = load_intrinsics(self.cfg.get("intrinsics_file", ""))
        self.camera = camera_from_config(self.cfg, self.K)
        if not self.camera.has_pose:
            self.get_logger().warn("No intrinsics/camera pose: tag heights are ignored, so raised tags (robot, "
                                   "blocks) are placed too far from the centre of the image.")
        self.empty = cv2.imread(self.cfg["empty_floor_image"]) if self.cfg.get("empty_floor_image") else None
        self.detector = make_detector(self.cfg.get("dictionary", "DICT_4X4_50"))

        self.grid_pub = self.create_publisher(OccupancyGrid, "occupancy_grid", LATCHED)
        self.objects_pub = self.create_publisher(ObjectPoseArray, "objects", LATCHED)
        self.robot_pub = self.create_publisher(PoseStamped, "overhead_robot", 10)
        self.create_service(Trigger, "refresh_map", self.on_refresh)
        self.create_timer(float(self.cfg.get("capture_period", 10.0)), self.capture)
        self.capture()

    def grab(self):
        camera = cv2.VideoCapture(int(self.cfg["device"]))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg["width"])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["height"])
        for _ in range(5):            # let auto-exposure settle
            ok, frame = camera.read()
        camera.release()
        if not ok:
            raise RuntimeError("no frame from the overhead camera")
        return cv2.undistort(frame, self.K, self.dist) if self.K is not None else frame

    def on_refresh(self, request, response):
        try:
            response.message = self.capture()
            response.success = True
        except RuntimeError as error:
            response.success, response.message = False, str(error)
        return response

    def capture(self):
        frame = self.grab()
        result = process_frame(frame, self.camera, self.detector, self.objects_cfg, self.cfg["grid"], self.empty)
        stamp = self.get_clock().now().to_msg()

        objects = ObjectPoseArray()
        objects.header.stamp, objects.header.frame_id = stamp, "map"
        for o in result["objects"]:
            obj = ObjectPose(name=o["name"], kind=o["kind"], tag_id=o["tag_id"], confidence=1.0)
            obj.pose.position.x, obj.pose.position.y = o["x"], o["y"]
            obj.pose.orientation.z, obj.pose.orientation.w = yaw_to_quaternion_zw(o["yaw"])
            objects.objects.append(obj)
        self.objects_pub.publish(objects)

        if result["robot"] is not None:
            x, y, yaw = result["robot"]
            robot = PoseStamped()
            robot.header = objects.header
            robot.pose.position.x, robot.pose.position.y = x, y
            robot.pose.orientation.z, robot.pose.orientation.w = yaw_to_quaternion_zw(yaw)
            self.robot_pub.publish(robot)

        g, grid = self.cfg["grid"], result["grid"]
        msg = OccupancyGrid()
        msg.header.stamp, msg.header.frame_id = stamp, "map"
        msg.info.resolution = float(g["resolution"])
        msg.info.height, msg.info.width = grid.shape
        msg.info.origin.position.x, msg.info.origin.position.y = map(float, g["origin"])
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.grid_pub.publish(msg)
        summary = f"{len(objects.objects)} objects: {[o.name for o in objects.objects]}"
        self.get_logger().info(summary)
        return summary


def main():
    rclpy.init()
    node = OverheadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
