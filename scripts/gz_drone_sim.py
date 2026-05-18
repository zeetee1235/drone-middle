#!/usr/bin/env python3
"""
gz_drone_sim.py — Standalone drone motion simulator for Gazebo.

In standalone Gazebo mode (no PX4 SITL) the sprint_camera_rig model is
static. This script makes it fly by:

  1. Subscribing to /fmu/in/trajectory_setpoint  (NED velocity from the
     px4_offboard_control node)
  2. Integrating velocity at 50 Hz to keep a simulated world-frame position
  3. Calling "gz service /world/<world>/set_pose" at ~20 Hz to teleport the
     model — so the downward camera sees the ground moving in real time

Frame conventions
  NED (PX4)  →  Gazebo world (ENU-ish, Y-north)
    vx (north) → gz_y
    vy (east)  → gz_x
    vz (down)  → -gz_z

Start position: vertiport = world (2, 19, 2.0)

Usage:
    python3 scripts/gz_drone_sim.py
    python3 scripts/gz_drone_sim.py --world sprint_grid_world_px4 --model sprint_camera_rig
"""

import argparse
import math
import subprocess
import sys
import threading
import time

# ── start position ────────────────────────────────────────────────────────────
START_X =  2.0   # world x  (east,  vertiport)
START_Y = 19.0   # world y  (north, vertiport)
START_Z =  0.3   # world z  (up,    near-ground start — drone takes off to TARGET_Z)
TARGET_Z = 2.0   # target cruise altitude (m AGL)

# ── bounds (world frame) ──────────────────────────────────────────────────────
BOUND_X = (0.0, 32.0)
BOUND_Y = (0.0, 23.0)
BOUND_Z = (0.05, 5.0)

UPDATE_HZ   = 50   # velocity integration rate
GZ_MOVE_HZ  = 20   # gz service call rate (every N integration ticks)
GZ_MOVE_DIV = UPDATE_HZ // GZ_MOVE_HZ

# ── rclpy import ──────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
except ImportError as e:
    sys.exit(
        f"Import error: {e}\n"
        "Source ROS2 and the workspace before running:\n"
        "  source /opt/ros/jazzy/setup.bash\n"
        "  source install/setup.bash"
    )


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GzDroneSim(Node):
    def __init__(self, world: str, model: str):
        super().__init__("gz_drone_sim")
        self._world = world
        self._model = model

        # Simulated world-frame position
        self._x = START_X
        self._y = START_Y
        self._z = START_Z

        # Latest NED velocity (m/s); guarded by lock
        self._vx_ned = 0.0   # north
        self._vy_ned = 0.0   # east
        self._vz_ned = 0.0   # down (neg = up)
        self._vel_lock = threading.Lock()

        # Mission state — only integrate when actively flying
        self._flying = False

        self.create_subscription(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self._on_setpoint,
            10,
        )
        self.create_subscription(
            String,
            "/mission/state",
            self._on_state,
            10,
        )

        # Publish simulated VehicleLocalPosition so the control node gets
        # altitude feedback without a real PX4 SITL.
        _px4_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub_local_pos = self.create_publisher(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            _px4_qos,
        )

        # Publish exact pose in grid frame so standalone mode skips visual odometry
        self._pub_pose_grid = self.create_publisher(
            PoseStamped,
            "/localization/pose_grid",
            10,
        )

        dt = 1.0 / UPDATE_HZ
        self._tick = 0
        self.create_timer(dt, self._step)

        self.get_logger().info(
            f"gz_drone_sim: model='{model}' world='{world}' "
            f"start=({START_X}, {START_Y}, {START_Z})"
        )
        # Move to start position immediately
        self._gz_set_pose(self._x, self._y, self._z)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_setpoint(self, msg: TrajectorySetpoint):
        with self._vel_lock:
            # velocity array is NED; NaN means "not controlled on this axis"
            vx = msg.velocity[0] if not math.isnan(msg.velocity[0]) else 0.0
            vy = msg.velocity[1] if not math.isnan(msg.velocity[1]) else 0.0
            vz = msg.velocity[2] if not math.isnan(msg.velocity[2]) else 0.0
            self._vx_ned = vx
            self._vy_ned = vy
            self._vz_ned = vz

    def _on_state(self, msg: String):
        active = msg.data not in ("INIT", "LANDED", "ABORT", "")
        if active != self._flying:
            self._flying = active
            self.get_logger().info(
                f"gz_drone_sim: {'flying' if active else 'holding'} "
                f"(mission_state={msg.data})"
            )

    # ── integration + gz move ─────────────────────────────────────────────────

    def _step(self):
        dt = 1.0 / UPDATE_HZ
        if self._flying:
            with self._vel_lock:
                # NED → world ENU conversion
                self._x += self._vy_ned * dt   # east  = NED y
                self._y += self._vx_ned * dt   # north = NED x
                self._z += -self._vz_ned * dt  # up    = -NED z

            self._x = _clamp(self._x, *BOUND_X)
            self._y = _clamp(self._y, *BOUND_Y)
            self._z = _clamp(self._z, *BOUND_Z)

        self._tick += 1
        if self._tick % GZ_MOVE_DIV == 0:
            self._gz_set_pose(self._x, self._y, self._z)
            self._publish_local_pos()
            self._publish_pose_grid()

    def _gz_set_pose(self, x: float, y: float, z: float):
        # gz service is fire-and-forget at low frequency; use a daemon thread
        # so slow gz calls don't block the ROS2 timer.
        req = (
            f'name: "{self._model}" '
            f'position: {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
            f'orientation: {{w: 1.0 x: 0.0 y: 0.0 z: 0.0}}'
        )
        t = threading.Thread(
            target=self._run_gz_service,
            args=(self._world, req),
            daemon=True,
        )
        t.start()

    def _publish_pose_grid(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "grid"
        # Grid frame origin = vertiport = Gazebo (START_X, START_Y)
        msg.pose.position.x = self._x - START_X   # east from vertiport
        msg.pose.position.y = self._y - START_Y   # north from vertiport (neg = south)
        msg.pose.position.z = self._z
        msg.pose.orientation.w = 1.0
        self._pub_pose_grid.publish(msg)

    def _publish_local_pos(self):
        msg = VehicleLocalPosition()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.z_valid = True
        msg.z = -float(self._z)  # NED: z = -altitude
        self._pub_local_pos.publish(msg)

    @staticmethod
    def _run_gz_service(world: str, req: str):
        try:
            subprocess.run(
                [
                    "gz", "service",
                    "-s", f"/world/{world}/set_pose",
                    "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "80",
                    "--req", req,
                ],
                capture_output=True,
                timeout=0.5,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass


def main():
    parser = argparse.ArgumentParser(description="Standalone Gazebo drone motion simulator")
    parser.add_argument("--world", default="sprint_grid_world",
                        help="Gazebo world name (default: sprint_grid_world)")
    parser.add_argument("--model", default="sprint_camera_rig",
                        help="Model name to teleport (default: sprint_camera_rig)")
    args = parser.parse_args()

    rclpy.init()
    node = GzDroneSim(world=args.world, model=args.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
