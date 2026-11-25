"""Task parser node: turns a typed request into a validated TaskSpec.

Subscribes:
    task_request   std_msgs/String: the request, for example "bring the red block to the tray"
    objects        roomrunner_interfaces/ObjectPoseArray: what the overhead camera sees (names and kinds)
Publishes:
    task_spec      roomrunner_interfaces/TaskSpec: the steps (go_to, pick, place) and the number of re-plans
Parameters:
    mode           "mock" (rule-based, no network; default), "direct" (LLM writes the steps) or
                   "llmp" (LLM writes a PDDL goal, the planner in pddl.py finds the steps)
    places_file    YAML with a "places" section of named places, such as home (fake_room.yaml)
    max_replans    how often the LLM may plan again after the validator rejects its plan (default 3)

The LLM provider and key come from the environment (.env), see llm_planner.make_llm. An empty TaskSpec
(no steps) tells the mission FSM that no valid plan was found.
"""
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from roomrunner_interfaces.msg import ObjectPoseArray, TaskSpec, TaskStep
from roomrunner_planning import llm_planner
from roomrunner_planning.mock_parser import parse
from roomrunner_planning.validator import validate


class TaskParser(Node):
    def __init__(self):
        super().__init__("task_parser")
        self.declare_parameter("mode", "mock")
        self.declare_parameter("places_file", "")
        self.declare_parameter("max_replans", 3)
        self.places = set()
        if self.get_parameter("places_file").value:
            with open(self.get_parameter("places_file").value) as f:
                self.places = set(yaml.safe_load(f).get("places", {}))
        self.objects = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ObjectPoseArray, "objects", self.on_objects, latched)
        self.create_subscription(String, "task_request", self.on_request, 10)
        self.spec_pub = self.create_publisher(TaskSpec, "task_spec", 10)
        self.llm = None
        self.get_logger().info(f"Task parser ready ({self.get_parameter('mode').value} mode).")

    def on_objects(self, msg):
        self.objects = {o.name: o.kind for o in msg.objects}

    def on_request(self, msg):
        mode = self.get_parameter("mode").value
        spec = TaskSpec(request=msg.data, planner=mode)
        spec.header.stamp = self.get_clock().now().to_msg()
        try:
            if mode == "mock":
                steps, replans = parse(msg.data), 0
                errors = validate(steps, self.objects, self.places)
                if errors:
                    raise ValueError("; ".join(errors))
            else:
                if self.llm is None:
                    self.llm = llm_planner.make_llm(self.objects, self.places)
                steps, replans = llm_planner.plan(msg.data, self.objects, self.places, self.llm, mode=mode,
                                                  max_replans=self.get_parameter("max_replans").value)
            spec.steps = [TaskStep(skill=s, target=t) for s, t in steps]
            spec.replans = replans
            self.get_logger().info(f"{msg.data!r} -> " + ", ".join(f"{s}({t})" for s, t in steps)
                                   + f" ({replans} re-plans)")
        except Exception as error:   # any failure (parse, validation, network) -> empty TaskSpec
            self.get_logger().warn(f"No valid plan for {msg.data!r}: {error}")
        self.spec_pub.publish(spec)


def main():
    rclpy.init()
    node = TaskParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
