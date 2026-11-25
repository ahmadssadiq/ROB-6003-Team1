"""Task parser node: turns a typed request into a TaskSpec.

Subscribes:
    task_request   std_msgs/String: the request, for example "bring the red block to the tray"
Publishes:
    task_spec      roomrunner_interfaces/TaskSpec: the steps (go_to, pick, place)
Parameters:
    mode           "mock" (rule-based, no network; default), later "direct" and "llmp"

Only mock mode exists so far. The LLM versions (direct steps, and LLM+P with a PDDL planner) and the
validator are the next steps of the planning module.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from roomrunner_interfaces.msg import TaskSpec, TaskStep
from roomrunner_planning.mock_parser import parse


class TaskParser(Node):
    def __init__(self):
        super().__init__("task_parser")
        self.declare_parameter("mode", "mock")
        self.spec_pub = self.create_publisher(TaskSpec, "task_spec", 10)
        self.create_subscription(String, "task_request", self.on_request, 10)
        self.get_logger().info(f"Task parser ready ({self.get_parameter('mode').value} mode).")

    def on_request(self, msg):
        spec = TaskSpec(request=msg.data, planner=self.get_parameter("mode").value)
        spec.header.stamp = self.get_clock().now().to_msg()
        try:
            spec.steps = [TaskStep(skill=skill, target=target) for skill, target in parse(msg.data)]
            self.get_logger().info(f"{msg.data!r} -> " + ", ".join(f"{s.skill}({s.target})" for s in spec.steps))
        except ValueError as error:
            self.get_logger().warn(f"Could not parse {msg.data!r}: {error}")
        self.spec_pub.publish(spec)   # an empty TaskSpec tells the mission FSM that parsing failed


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
