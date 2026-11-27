"""Mission state machine node: runs each TaskSpec with the navigation and pick/place skills.

The states and transitions are in fsm.py and the mission logic in mission.py (both tested without ROS);
this node only connects them to ROS.

Subscribes:
    task_spec       roomrunner_interfaces/TaskSpec
    objects         roomrunner_interfaces/ObjectPoseArray (map frame)
    robot_pose      geometry_msgs/PoseStamped (map frame)
    occupancy_grid  nav_msgs/OccupancyGrid (map frame), for the A* reachability check before a mission starts
Publishes:
    mission_status  roomrunner_interfaces/MissionStatus, on every state change and once a second
    planned_path    nav_msgs/Path (map frame), the path of the current leg
Uses:
    refresh_map     std_srvs/Trigger (overhead camera; skipped if it is not running, as in stub mode)
    plan_path       roomrunner_interfaces/PlanPath (service)
    navigate_to     roomrunner_interfaces/NavigateTo (action)
    pick_place      roomrunner_interfaces/PickPlace (action)
Parameters:
    places_file         YAML with a "places" section of named poses, such as home
    standoff            m between the base centre and an object when the base stops next to it (0.30)
    position_tolerance  m (0.03)        yaw_tolerance  rad (0.05)
    max_retries         retries per mission after a failed leg (2)
    pose_timeout        s without robot_pose during navigation before localization counts as lost (2.0)
    robot_radius        m: obstacles grown by this much in the reachability check, as in path_planner (0.20)
"""
import math
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from roomrunner_interfaces.action import NavigateTo, PickPlace
from roomrunner_interfaces.msg import MissionStatus, ObjectPoseArray, TaskSpec
from roomrunner_interfaces.srv import PlanPath
from roomrunner_planning.fsm import MissionStateMachine
from roomrunner_planning.mission import run_mission
from roomrunner_planning.reachability import Grid
from roomrunner_planning.validator import validate


class Timeout(Exception):
    pass


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_stamped(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def approach_pose(robot_xy, target_xy, standoff):
    """Where the base stops next to a target: `standoff` metres short of it, facing it."""
    dx, dy = target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]
    yaw = math.atan2(dy, dx)
    return target_xy[0] - standoff * math.cos(yaw), target_xy[1] - standoff * math.sin(yaw), yaw


def wait_for(future, timeout):
    """Block this worker thread until a future completed by the executor's threads is done."""
    start = time.monotonic()
    while not future.done():
        if time.monotonic() - start > timeout:
            raise Timeout()
        time.sleep(0.02)
    return future.result()


class MissionFSM(Node):
    def __init__(self):
        super().__init__("mission_fsm")
        for name, default in (("places_file", ""), ("standoff", 0.30), ("position_tolerance", 0.03),
                              ("yaw_tolerance", 0.05), ("max_retries", 2), ("pose_timeout", 2.0),
                              ("robot_radius", 0.20)):
            self.declare_parameter(name, default)
        self.places = {}
        if self.get_parameter("places_file").value:
            with open(self.get_parameter("places_file").value) as f:
                self.places = yaml.safe_load(f).get("places", {})

        self.objects, self.robot, self.robot_time = {}, None, 0.0
        self.grid = None
        self.path = None
        self.state, self.detail, self.leg, self.busy = "IDLE", "", -1, False
        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskSpec, "task_spec", self.on_task, 10, callback_group=group)
        self.create_subscription(ObjectPoseArray, "objects", self.on_objects, latched, callback_group=group)
        self.create_subscription(PoseStamped, "robot_pose", self.on_pose, 10, callback_group=group)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.on_grid, latched, callback_group=group)
        self.status_pub = self.create_publisher(MissionStatus, "mission_status", 10)
        self.path_pub = self.create_publisher(Path, "planned_path", 10)
        self.refresh_client = self.create_client(Trigger, "refresh_map", callback_group=group)
        self.plan_client = self.create_client(PlanPath, "plan_path", callback_group=group)
        self.nav_client = ActionClient(self, NavigateTo, "navigate_to", callback_group=group)
        self.pick_client = ActionClient(self, PickPlace, "pick_place", callback_group=group)
        self.create_timer(1.0, self.publish_status, callback_group=group)
        self.get_logger().info("Mission FSM ready. Waiting for a task_spec.")

    # ---- inputs and status ---------------------------------------------------------------------
    def on_objects(self, msg):
        self.objects = {o.name: o for o in msg.objects}

    def on_grid(self, msg):
        info = msg.info
        self.grid = Grid.from_occupancy(msg.data, info.width, info.height, info.resolution,
                                        (info.origin.position.x, info.origin.position.y),
                                        self.get_parameter("robot_radius").value)

    def on_pose(self, msg):
        self.robot = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose))
        self.robot_time = time.monotonic()

    def on_task(self, spec):
        if self.busy:
            self.get_logger().warn("A mission is already running; request ignored.")
            return
        self.busy = True
        steps = [(s.skill, s.target) for s in spec.steps]
        threading.Thread(target=self.run, args=(spec.request, steps), daemon=True).start()

    def on_state(self, state, detail, leg):
        self.state, self.detail, self.leg = state, detail, leg
        self.get_logger().info(f"[{state}] {detail}")
        self.publish_status()

    def publish_status(self):
        msg = MissionStatus(state=self.state, step_index=self.leg, detail=self.detail, success=self.state == "DONE")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)

    def run(self, request, steps):
        try:
            final, detail = run_mission(steps, self, MissionStateMachine(self.get_parameter("max_retries").value),
                                        self.on_state, request)
            self.get_logger().info(f"{request!r}: {final}, {detail}")
        except Exception as error:   # never leave the robot in a half-run mission without saying so
            self.on_state("FAILED", f"internal error: {error}", self.leg)
        finally:
            time.sleep(1.0)
            self.on_state("IDLE", "", -1)
            self.busy = False

    # ---- skills (see mission.py) ---------------------------------------------------------------
    def refresh_map(self):
        if not self.refresh_client.service_is_ready():
            return   # no overhead camera (stub mode): the latched map is used as it is
        try:
            wait_for(self.refresh_client.call_async(Trigger.Request()), timeout=10.0)
        except Timeout:
            self.get_logger().warn("refresh_map timed out; using the last map")

    def check(self, steps):
        kinds = {name: obj.kind for name, obj in self.objects.items()}
        reachable = None
        if self.grid is not None and self.robot is not None:   # otherwise planning reports a missing path later
            positions = {name: (p["x"], p["y"]) for name, p in self.places.items()}
            positions.update({name: (o.pose.position.x, o.pose.position.y) for name, o in self.objects.items()})
            reachable = self.grid.reachable_function(self.robot[:2], positions)
        return validate(steps, kinds, set(self.places), reachable)

    def target_pose(self, name):
        if name in self.objects:
            p = self.objects[name].pose
            return (p.position.x, p.position.y), yaw_of(p), True
        place = self.places[name]
        return (place["x"], place["y"]), place["yaw"], False

    def plan(self, target):
        if self.robot is None:
            return False, "no robot pose"
        xy, yaw, is_object = self.target_pose(target)
        goal = approach_pose(self.robot[:2], xy, self.get_parameter("standoff").value) if is_object else (*xy, yaw)
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            return False, "plan_path service not available"
        try:
            reply = wait_for(self.plan_client.call_async(
                PlanPath.Request(start=pose_stamped(*self.robot), goal=pose_stamped(*goal))), timeout=10.0)
        except Timeout:
            return False, "plan_path timed out"
        if not reply.success:
            return False, reply.message
        self.path = reply.path
        self.path_pub.publish(reply.path)
        return True, f"{reply.length:.2f} m"

    def navigate(self, target, on_arrived):
        goal = NavigateTo.Goal(path=self.path, position_tolerance=self.get_parameter("position_tolerance").value,
                               yaw_tolerance=self.get_parameter("yaw_tolerance").value)
        arrived = []

        def on_feedback(message):
            if not arrived and message.feedback.distance_remaining < goal.position_tolerance:
                arrived.append(True)
                on_arrived()

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return "nav_failed"
        try:
            handle = wait_for(self.nav_client.send_goal_async(goal, feedback_callback=on_feedback), timeout=5.0)
        except Timeout:
            return "nav_failed"
        if not handle.accepted:
            return "nav_failed"
        result_future = handle.get_result_async()
        pose_timeout, start = self.get_parameter("pose_timeout").value, time.monotonic()
        while not result_future.done():
            if time.monotonic() - self.robot_time > pose_timeout:
                handle.cancel_goal_async()
                return "localization_lost"
            if time.monotonic() - start > 300.0:
                handle.cancel_goal_async()
                return "nav_failed"
            time.sleep(0.05)
        return "aligned" if result_future.result().result.success else "nav_failed"

    def manipulate(self, skill, target):
        xy, yaw, _ = self.target_pose(target)
        goal = PickPlace.Goal(mode=skill, object_name=target, target=pose_stamped(*xy, yaw))
        if not self.pick_client.wait_for_server(timeout_sec=5.0):
            return False
        try:
            handle = wait_for(self.pick_client.send_goal_async(goal), timeout=5.0)
            return handle.accepted and wait_for(handle.get_result_async(), timeout=120.0).result.success
        except Timeout:
            return False


def main():
    rclpy.init()
    node = MissionFSM()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
