"""Success = cube stays inside the goal zone (xy circle) and on the table for hold_sec.

Subscribes: poses (tf2_msgs/TFMessage, bridged from Gazebo dynamic_pose/info)
Publishes:  task/success (std_msgs/Bool, 10 Hz)
Keep goal_xy / xy_radius consistent with the green marker in spawn_robot/worlds/pushcube.sdf.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


class SuccessChecker(Node):
    def __init__(self):
        super().__init__("success_checker")
        self.declare_parameter("cube_frame", "cube")
        self.declare_parameter("goal_xy", [0.55, -0.10])
        self.declare_parameter("xy_radius", 0.07)
        self.declare_parameter("z_max", 0.06)
        self.declare_parameter("hold_sec", 1.0)
        self.pub = self.create_publisher(Bool, "task/success", 10)
        self.create_subscription(TFMessage, "poses", self.on_poses, 10)
        self.inside_since = None
        self.success = False
        self.seen = False
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        self.pub.publish(Bool(data=self.success))
        if not self.seen:
            self.get_logger().warning(
                "no pose for the cube yet - check `ros2 topic echo /poses --once` "
                "and set cube_frame to that child_frame_id",
                throttle_duration_sec=5.0)

    def on_poses(self, msg):
        name = self.get_parameter("cube_frame").value
        for tf in msg.transforms:
            child = tf.child_frame_id
            if child == name or child.startswith(name + "/") or child.startswith(name + "::"):
                self.seen = True
                position = tf.transform.translation
                goal_x, goal_y = self.get_parameter("goal_xy").value
                inside = (
                    math.hypot(position.x - goal_x, position.y - goal_y)
                    < self.get_parameter("xy_radius").value
                    and position.z < self.get_parameter("z_max").value
                )
                if inside:
                    if self.inside_since is None:
                        self.inside_since = self.now()
                    self.success = (
                        self.now() - self.inside_since >= self.get_parameter("hold_sec").value
                    )
                else:
                    self.inside_since = None
                    self.success = False
                return


def main():
    rclpy.init()
    node = SuccessChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
