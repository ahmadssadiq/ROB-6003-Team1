"""Mission state machine: runs a TaskSpec step by step with the navigation and pick/place skills.

Subscribes:
    task_spec       roomrunner_interfaces/TaskSpec
    objects         roomrunner_interfaces/ObjectPoseArray (map frame)
    robot_pose      geometry_msgs/PoseStamped (map frame)
Publishes:
    mission_status  roomrunner_interfaces/MissionStatus, on every state change and once a second
    planned_path    nav_msgs/Path (map frame), the path of the current leg
Uses:
    plan_path       roomrunner_interfaces/PlanPath (service)
    navigate_to     roomrunner_interfaces/NavigateTo (action)
    pick_place      roomrunner_interfaces/PickPlace (action)
Parameters:
    places_file         YAML with a "places" section of named poses, such as home (fake_room.yaml)
    standoff            m between the base centre and an object when the base stops next to it (0.30)
    position_tolerance  m (default 0.03)
    yaw_tolerance       rad (default 0.05)

States: IDLE -> PARSE_TASK -> PLAN_TO_PICKUP / PLAN_TO_DROPOFF -> NAVIGATE -> ALIGN -> PICK / PLACE -> ...
-> DONE, or FAILED. This first version runs the happy path; RETRY and the failure handling come next.
"""
import math
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from roomrunner_interfaces.action import NavigateTo, PickPlace
from roomrunner_interfaces.msg import MissionStatus, ObjectPoseArray, TaskSpec
from roomrunner_interfaces.srv import PlanPath


class SkillFailed(Exception):
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


def wait_for(future, timeout=None):
    """Block this worker thread until a future from the executor's threads is done."""
    start = time.monotonic()
    while not future.done():
        if timeout is not None and time.monotonic() - start > timeout:
            raise SkillFailed("timed out")
        time.sleep(0.05)
    return future.result()


class MissionFSM(Node):
    def __init__(self):
        super().__init__("mission_fsm")
        self.declare_parameter("places_file", "")
        self.declare_parameter("standoff", 0.30)
        self.declare_parameter("position_tolerance", 0.03)
        self.declare_parameter("yaw_tolerance", 0.05)
        self.places = {}
        if self.get_parameter("places_file").value:
            with open(self.get_parameter("places_file").value) as f:
                self.places = yaml.safe_load(f).get("places", {})

        self.objects, self.robot = {}, None
        self.state, self.step_index, self.detail, self.busy = "IDLE", -1, "", False
        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskSpec, "task_spec", self.on_task, 10, callback_group=group)
        self.create_subscription(ObjectPoseArray, "objects", self.on_objects, latched, callback_group=group)
        self.create_subscription(PoseStamped, "robot_pose", self.on_pose, 10, callback_group=group)
        self.status_pub = self.create_publisher(MissionStatus, "mission_status", 10)
        self.path_pub = self.create_publisher(Path, "planned_path", 10)
        self.plan_client = self.create_client(PlanPath, "plan_path", callback_group=group)
        self.nav_client = ActionClient(self, NavigateTo, "navigate_to", callback_group=group)
        self.pick_client = ActionClient(self, PickPlace, "pick_place", callback_group=group)
        self.create_timer(1.0, self.publish_status, callback_group=group)
        self.get_logger().info("Mission FSM ready. Waiting for a task_spec.")

    # ---- inputs -------------------------------------------------------------------------------
    def on_objects(self, msg):
        self.objects = {o.name: o for o in msg.objects}

    def on_pose(self, msg):
        self.robot = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose))

    def on_task(self, spec):
        if self.busy:
            self.get_logger().warn("A mission is already running; request ignored.")
            return
        self.busy = True
        threading.Thread(target=self.run, args=(spec,), daemon=True).start()

    # ---- status -------------------------------------------------------------------------------
    def set_state(self, state, detail=""):
        self.state, self.detail = state, detail
        self.get_logger().info(f"[{state}] {detail}")
        self.publish_status()

    def publish_status(self):
        msg = MissionStatus(state=self.state, step_index=self.step_index, detail=self.detail,
                            success=self.state == "DONE")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)

    # ---- the mission --------------------------------------------------------------------------
    def run(self, spec):
        try:
            self.step_index = -1
            self.set_state("PARSE_TASK", spec.request)
            if not spec.steps:
                raise SkillFailed("the request could not be parsed")
            for step in spec.steps:
                if step.target not in self.objects and step.target not in self.places:
                    raise SkillFailed(f"object or place not found: {step.target}")
            holding = None
            for index, step in enumerate(spec.steps):
                self.step_index = index
                if step.skill == "go_to":
                    leg = "PLAN_TO_DROPOFF" if holding else "PLAN_TO_PICKUP"
                    self.go_to(step.target, leg)
                elif step.skill in ("pick", "place"):
                    self.pick_or_place(step.skill, step.target)
                    holding = step.target if step.skill == "pick" else None
                else:
                    raise SkillFailed(f"unknown skill {step.skill!r}")
            self.set_state("DONE", f"{spec.request!r} completed")
        except SkillFailed as error:
            self.set_state("FAILED", str(error))
        finally:
            self.busy = False
            time.sleep(1.0)
            self.step_index = -1
            self.set_state("IDLE")

    def target_xy_yaw(self, name):
        if name in self.objects:
            p = self.objects[name].pose
            return (p.position.x, p.position.y), yaw_of(p), True
        place = self.places[name]
        return (place["x"], place["y"]), place["yaw"], False

    def go_to(self, target, leg):
        if self.robot is None:
            raise SkillFailed("no robot pose (localization lost)")
        self.set_state(leg, f"to {target}")
        xy, yaw, is_object = self.target_xy_yaw(target)
        if is_object:   # stop next to an object, facing it
            gx, gy, gyaw = approach_pose(self.robot[:2], xy, self.get_parameter("standoff").value)
        else:           # drive onto a named place
            gx, gy, gyaw = xy[0], xy[1], yaw
        request = PlanPath.Request(start=pose_stamped(*self.robot), goal=pose_stamped(gx, gy, gyaw))
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            raise SkillFailed("plan_path service not available")
        plan = wait_for(self.plan_client.call_async(request), timeout=10.0)
        if not plan.success:
            raise SkillFailed(f"no valid path to {target}: {plan.message}")
        self.path_pub.publish(plan.path)

        self.set_state("NAVIGATE", f"to {target}, {plan.length:.2f} m")
        goal = NavigateTo.Goal(path=plan.path, position_tolerance=self.get_parameter("position_tolerance").value,
                               yaw_tolerance=self.get_parameter("yaw_tolerance").value)

        def on_feedback(message):
            if self.state == "NAVIGATE" and message.feedback.distance_remaining < goal.position_tolerance:
                self.set_state("ALIGN", f"at {target}, turning to the goal heading")

        result = self.run_action(self.nav_client, "navigate_to", goal, on_feedback, timeout=180.0)
        if not result.success:
            raise SkillFailed(f"navigation to {target} failed: {result.message}")
        self.set_state("ALIGN", f"aligned and stopped at {target}, error {100 * result.final_position_error:.1f} cm")

    def pick_or_place(self, skill, target):
        self.set_state("PICK" if skill == "pick" else "PLACE", target)
        xy, yaw, _ = self.target_xy_yaw(target)
        goal = PickPlace.Goal(mode=skill, object_name=target, target=pose_stamped(xy[0], xy[1], yaw))
        result = self.run_action(self.pick_client, "pick_place", goal, None, timeout=60.0)
        if not result.success:
            raise SkillFailed(f"{skill} {target} failed: {result.message}")

    def run_action(self, client, name, goal, on_feedback, timeout):
        if not client.wait_for_server(timeout_sec=5.0):
            raise SkillFailed(f"action server {name} not available")
        handle = wait_for(client.send_goal_async(goal, feedback_callback=on_feedback), timeout=5.0)
        if not handle.accepted:
            raise SkillFailed("goal rejected")
        return wait_for(handle.get_result_async(), timeout=timeout).result


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
