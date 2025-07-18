#!/usr/bin/env python3
"""
Destination Controller with REST API (Flask)
==========================================
This ROS 2 node exposes a tiny web‑service so you can send navigation goals as
JSON instead of typing at the CLI.

  • POST /goal   {"destination": "fridge"}
                 {"x": -4.0, "y": 0.0, "z": 0.0, "yaw": 1.57}
  • GET  /health  → "ok" (liveness probe)

Usage ────────────────────────────────────────────────────────────────────────
    # inside your ROS 2 workspace python env
    pip install flask         # first time only
    ros2 run flowbit_nav robot_server  # or `python3 robot_server.py`

For production use a proper WSGI server (gunicorn/uvicorn) instead of the
built‑in Flask development server.
"""
from __future__ import annotations

import signal
import subprocess
import sys
import threading
from typing import Any, Dict, Tuple

from flask import Flask, jsonify, request
from flask_cors import CORS  
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 node
# ─────────────────────────────────────────────────────────────────────────────
class DestinationController(Node):
    def __init__(self) -> None:
        super().__init__("destination_controller_web")

        # Named way‑points reachable via {"destination": "<name>"}
        self.destinations: Dict[str, Tuple[float, float, float, float]] = {
            "fridge": (-4.01952, 0.0, -1.50533, 1.5),
            "home": (2.5, 0.0117438, 1.1187, 1.5),
            "server": (-0.716533, 0.0, -1.45803, 1.5),
            "water": (-3.1, 0.0, 0.54056, 1.5),
        }

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("DestinationController ready — waiting for REST goals…")

    # ───────── helpers ─────────
    def _stop_robot(self) -> None:
        """Publish zero velocity so the robot halts before a new goal."""
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Robot stopped (cmd_vel = 0).")

    def _send_goal(self, x: float, y: float, z: float, yaw: float) -> None:
        """Launch the goal publisher with the requested pose."""
        cmd = (
            "ros2 run flowbit_nav goal_pose_pub "
            f"--ros-args -p x:={x} -p y:={y} -p z:={z} -p yaw:={yaw}"
        )
        subprocess.Popen(cmd, shell=True)
        self.get_logger().info(f"Sent goal → x:{x}, y:{y}, z:{z}, yaw:{yaw}")

    # ───────── public API ─────────
    def dispatch(self, payload: Dict[str, Any]) -> Tuple[bool, str]:
        """Handle a JSON payload coming from the Flask route."""
        # Named destination
        if "destination" in payload:
            dest = str(payload["destination"]).strip().lower()
            if dest in self.destinations:
                self._stop_robot()
                self._send_goal(*self.destinations[dest])
                return True, f"Moving to '{dest}'"
            return False, (
                f"Unknown destination '{dest}'. Choose one of: "
                f"{', '.join(self.destinations.keys())}."
            )

        # Raw pose
        try:
            x = float(payload["x"])
            y = float(payload["y"])
            z = float(payload.get("z", 0.0))
            yaw = float(payload.get("yaw", 0.0))
        except (KeyError, ValueError):
            return False, (
                "Payload must contain either 'destination' or numeric "
                "'x', 'y', 'z', 'yaw' fields."
            )

        self._stop_robot()
        self._send_goal(x, y, z, yaw)
        return True, "Moving to specified coordinates"


# ─────────────────────────────────────────────────────────────────────────────
# Flask app (runs in the *main* thread)
# ─────────────────────────────────────────────────────────────────────────────
app = Flask(__name__)
CORS(app)
controller: DestinationController | None = None  # populated in main()


@app.route("/goal", methods=["POST"])
def goal() -> tuple:
    if controller is None:
        return jsonify({"success": False, "message": "Controller offline"}), 503
    success, msg = controller.dispatch(request.get_json(force=True))
    return jsonify({"success": success, "message": msg}), (200 if success else 400)


@app.route("/health", methods=["GET"])
def health() -> tuple:
    return "ok", 200


# ─────────────────────────────────────────────────────────────────────────────
# Graceful shutdown helpers
# ─────────────────────────────────────────────────────────────────────────────

def _shutdown(_sig, _frame) -> None:  # signal handler
    global controller
    if controller is not None:
        controller.get_logger().info("Shutting down gracefully…")
        controller.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


# ─────────────────────────────────────────────────────────────────────────────
# Entry‑point — usable by both `python robot_server.py` and ROS 2 `console_scripts`
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:  # noqa: D401  (simple name is required by ROS 2)
    """Spin ROS in a background thread and run Flask in the foreground."""
    global controller

    # Initialise ROS 2 + node
    rclpy.init()
    controller = DestinationController()

    # ROS executor (non‑blocking) in a daemon thread
    spin_th = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True, name="ros_spin")
    spin_th.start()

    # Ctrl‑C / SIGTERM make everything stop tidily
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _shutdown)

    # Blocking — replace with gunicorn/uvicorn behind Nginx for production
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()
