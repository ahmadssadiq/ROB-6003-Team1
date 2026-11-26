"""The simulated RoomRunner room: the course sim_node (lekiwi_ros) with our room added, plus the overhead camera.

Everything of the course sim_node works as usual (arm_commands, base_commands, arm_state, base_velocity,
true_pose, the front camera on camera/image, ...). In addition:
Publishes:
    overhead/image   sensor_msgs/Image (rgb8) from the camera above the arena, every overhead_camera.period s
Writes:
    empty_floor_file the overhead picture of the bare floor (everything but the floor moved out of view for
                     one picture), which overhead_node compares with to find obstacles
Parameters (besides sim_node's own; use scene:=aruco and map_file:=<Assignment 04 sim_room_map.yaml>):
    room_file        sim_room.yaml: obstacles, cuboids, tray, robot tag, overhead camera
    objects_file     objects.yaml: the cuboids' sizes and colours
    empty_floor_file where to save the empty-floor picture (sim.launch.py: ~/.ros/roomrunner_sim/...)

The room is put into the course scene by room_builder.install before the course node builds its simulator;
no course file is changed. Run in the conda environment (MuJoCo).
"""
import cv2
import mujoco
import rclpy
import yaml
from sensor_msgs.msg import Image

from roomrunner_sim import room_builder


def read_setup():
    """room_file and objects_file from this process's ROS arguments (before the simulator node exists)."""
    node = rclpy.create_node("room_sim_setup")
    node.declare_parameter("room_file", "")
    node.declare_parameter("objects_file", "")
    node.declare_parameter("empty_floor_file", "")
    paths = node.get_parameter("room_file").value, node.get_parameter("objects_file").value
    empty_floor = node.get_parameter("empty_floor_file").value
    node.destroy_node()
    if not all(paths):
        raise SystemExit("room_sim needs -p room_file:=... -p objects_file:=... (sim.launch.py passes them)")
    return [yaml.safe_load(open(p)) for p in paths] + [empty_floor]


def main():
    rclpy.init()
    room, objects, empty_floor = read_setup()
    room_builder.install(room, objects)
    from lekiwi_ros.sim_node import SimNode                # imported after install: it builds the scene

    class RoomSimNode(SimNode):
        def __init__(self):
            super().__init__()
            self.sim.camera_names.append("overhead")      # the camera room_builder added
            c = room["overhead_camera"]
            self.overhead_size = (c["width"], c["height"])
            self.overhead_pub = self.create_publisher(Image, "overhead/image", 2)
            self.create_timer(float(c["period"]), self.publish_overhead)
            if empty_floor:
                cv2.imwrite(empty_floor, cv2.cvtColor(self.empty_floor_picture(), cv2.COLOR_RGB2BGR))
                self.get_logger().info(f"Empty-floor picture: {empty_floor}")

        def empty_floor_picture(self):
            """The overhead view of the bare floor: for this one picture, every free body (the robot, the
            cuboids) and every fixed geom except the floor is moved 50 m down, then everything is put back."""
            model, data = self.sim.model, self.sim.data
            qpos, geom_pos = data.qpos.copy(), model.geom_pos.copy()
            for j in range(model.njnt):
                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    data.qpos[model.jnt_qposadr[j] + 2] -= 50.0
            fixed = model.geom_bodyid == 0
            fixed[model.geom("floor").id] = False
            model.geom_pos[fixed, 2] -= 50.0
            mujoco.mj_forward(model, data)
            try:
                return self.sim.get_image("overhead", *self.overhead_size)
            finally:
                data.qpos[:], model.geom_pos[:] = qpos, geom_pos
                mujoco.mj_forward(model, data)

        def publish_overhead(self):
            picture = self.sim.get_image("overhead", *self.overhead_size)
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "overhead_camera"
            msg.height, msg.width = picture.shape[0], picture.shape[1]
            msg.encoding, msg.step = "rgb8", picture.shape[1] * 3
            msg.data = picture.tobytes()
            self.overhead_pub.publish(msg)

    node = RoomSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.sim.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
