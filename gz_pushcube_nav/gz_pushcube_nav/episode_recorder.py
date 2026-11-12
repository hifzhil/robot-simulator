"""Records episodes to HDF5: joint state + overhead camera + commanded joint velocities.

Control with teleop events (keys b / n / m in keyboard_teleop) or:
  ros2 topic pub --once /teleop/event std_msgs/msg/String "{data: start}"

File layout
  observation/state (N,6) float32   observation/image (N,H,W,3) uint8
  action (N,6) float32              timestamp (N,) float64      sync_gap (N,) float32
  attrs: success, fps, task, joint_names (json), episode_index, dropped_ticks
"""
import glob
import json
import os
import re

import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64MultiArray, String

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def image_to_array(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported encoding {msg.encoding}")
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = image[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return (image[:, :, ::-1] if msg.encoding == "bgr8" else image).copy()


class EpisodeRecorder(Node):
    def __init__(self):
        super().__init__("episode_recorder")
        self.declare_parameter("output_dir", "~/episodes_stage1")
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("task", "push the red cube into the green zone")
        self.declare_parameter("max_stale_sec", 0.3)
        self.out_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        os.makedirs(self.out_dir, exist_ok=True)
        self.fps = float(self.get_parameter("fps").value)

        self.joint = self.joint_t = None
        self.image = self.image_t = None
        self.cmd = self.cmd_t = None
        self.success = False
        self.recording = False
        self._reset()

        self.create_subscription(JointState, "joint_states", self.on_joint, 10)
        self.create_subscription(Image, "camera/image_raw", self.on_image, 5)
        self.create_subscription(
            Float64MultiArray, "/forward_velocity_controller/commands", self.on_cmd, 10)
        self.create_subscription(Bool, "task/success", self.on_success, 10)
        self.create_subscription(String, "teleop/event", self.on_event, 10)
        self.create_timer(1.0 / self.fps, self.tick)
        self.get_logger().info(f"recorder ready -> {self.out_dir}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _reset(self):
        self.times, self.state, self.actions, self.images, self.gaps = [], [], [], [], []
        self.dropped = 0

    def on_success(self, msg):
        self.success = msg.data

    def on_joint(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        try:
            self.joint = np.array(
                [msg.position[index[name]] for name in ARM_JOINTS], dtype=np.float32)
        except KeyError:
            return
        self.joint_t = stamp_sec(msg.header.stamp)

    def on_image(self, msg):
        try:
            self.image = image_to_array(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        self.image_t = stamp_sec(msg.header.stamp)

    def on_cmd(self, msg):
        if len(msg.data) < 6:
            return
        self.cmd = np.array(msg.data[:6], dtype=np.float32)
        self.cmd_t = self.now()

    def on_event(self, msg):
        if msg.data == "start" and not self.recording:
            self._reset()
            self.recording = True
            self.get_logger().info("episode started")
        elif msg.data in ("stop", "discard"):
            self.finish(discard=(msg.data == "discard"))

    def tick(self):
        if not self.recording:
            return
        if self.joint is None or self.image is None or self.cmd is None:
            self.dropped += 1
            return
        stamps = [self.joint_t, self.image_t, self.cmd_t]
        now = self.now()
        if now - min(stamps) > self.get_parameter("max_stale_sec").value:
            self.dropped += 1
            return
        self.times.append(now)
        self.state.append(self.joint.copy())
        self.images.append(self.image)
        self.actions.append(self.cmd.copy())
        self.gaps.append(max(stamps) - min(stamps))

    def next_index(self):
        found = []
        for path in glob.glob(os.path.join(self.out_dir, "episode_*.h5")):
            match = re.search(r"episode_(\d+)", os.path.basename(path))
            if match:
                found.append(int(match.group(1)))
        return max(found) + 1 if found else 0

    def finish(self, discard):
        if not self.recording:
            return
        self.recording = False
        count = len(self.times)
        if discard or count < 2:
            self.get_logger().info(f"episode discarded ({count} frames)")
            return
        index = self.next_index()
        path = os.path.join(self.out_dir, f"episode_{index:06d}.h5")
        height, width = self.images[0].shape[:2]
        with h5py.File(path, "w") as handle:
            handle.create_dataset("observation/state", data=np.stack(self.state))
            handle.create_dataset(
                "observation/image",
                data=np.stack(self.images),
                chunks=(1, height, width, 3),
                compression="gzip",
                compression_opts=4,
            )
            handle.create_dataset("action", data=np.stack(self.actions))
            handle.create_dataset("timestamp", data=np.array(self.times, dtype=np.float64))
            handle.create_dataset("sync_gap", data=np.array(self.gaps, dtype=np.float32))
            handle.attrs["success"] = bool(self.success)
            handle.attrs["fps"] = self.fps
            handle.attrs["task"] = self.get_parameter("task").value
            handle.attrs["joint_names"] = json.dumps(ARM_JOINTS)
            handle.attrs["episode_index"] = index
            handle.attrs["dropped_ticks"] = self.dropped
        self.get_logger().info(
            f"saved {path}: {count} frames, success={self.success}, dropped={self.dropped}")


def main():
    rclpy.init()
    node = EpisodeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
