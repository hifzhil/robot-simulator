"""Joint-space keyboard teleop for the UR5e (run in its own terminal).

Publishes
  /forward_velocity_controller/commands  std_msgs/Float64MultiArray  (6 joint velocities, rad/s, 50 Hz)
  /teleop/event                          std_msgs/String             start | stop | discard

Keys (each press moves that joint for `pulse_sec`; holding a key keeps it moving)
  joint 1: q / a      joint 2: w / s      joint 3: e / d
  joint 4: r / f      joint 5: t / g      joint 6: y / h
  [ / ]  slower / faster      space  stop all
  b  begin episode    n  save episode     m  discard episode
  Ctrl-C quit
"""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

JOINT_KEYS = {
    "q": (0, 1), "a": (0, -1), "w": (1, 1), "s": (1, -1), "e": (2, 1), "d": (2, -1),
    "r": (3, 1), "f": (3, -1), "t": (4, 1), "g": (4, -1), "y": (5, 1), "h": (5, -1),
}
EVENT_KEYS = {"b": "start", "n": "stop", "m": "discard"}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("speed", 0.3)
        self.declare_parameter("pulse_sec", 0.35)
        self.declare_parameter("rate_hz", 50.0)
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/forward_velocity_controller/commands", 10)
        self.event_pub = self.create_publisher(String, "teleop/event", 10)
        self.speed = float(self.get_parameter("speed").value)
        self.until = [0.0] * 6
        self.direction = [0] * 6
        self.last_pub = 0.0

    def handle_key(self, ch):
        now = time.monotonic()
        pulse = float(self.get_parameter("pulse_sec").value)
        if ch in JOINT_KEYS:
            joint, sign = JOINT_KEYS[ch]
            self.direction[joint] = sign
            self.until[joint] = now + pulse
        elif ch in EVENT_KEYS:
            self.event_pub.publish(String(data=EVENT_KEYS[ch]))
            print(f"\r[event] {EVENT_KEYS[ch]}          ")
        elif ch == "[":
            self.speed = max(0.05, self.speed * 0.8)
            print(f"\r[speed] {self.speed:.2f} rad/s     ")
        elif ch == "]":
            self.speed = min(1.0, self.speed * 1.25)
            print(f"\r[speed] {self.speed:.2f} rad/s     ")
        elif ch == " ":
            self.until = [0.0] * 6

    def publish(self, force_zero=False):
        now = time.monotonic()
        if not force_zero and now - self.last_pub < 1.0 / float(self.get_parameter("rate_hz").value):
            return
        self.last_pub = now
        vel = [
            0.0 if force_zero or now > self.until[joint] else self.direction[joint] * self.speed
            for joint in range(6)
        ]
        self.cmd_pub.publish(Float64MultiArray(data=vel))


def main():
    if not sys.stdin.isatty():
        raise SystemExit("keyboard_teleop needs an interactive terminal (use ros2 run, not ros2 launch)")
    rclpy.init()
    node = KeyboardTeleop()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(__doc__)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if ready:
                node.handle_key(sys.stdin.read(1))
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.publish(force_zero=True)
        node.destroy_node()
        rclpy.shutdown()
