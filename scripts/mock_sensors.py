#!/usr/bin/env python3
"""
mock_sensors.py — headless experiment support node.

Replaces PX4 SITL + Gazebo camera pipeline so the full mission loop runs
without any simulator.  Provides:

  1. VehicleAttitude        (identity quat, 50 Hz)
  2. BatteryStatus          (100 %, 10 Hz)
  3. TrajectorySetpoint     converted from /planner/velocity_profile (50 Hz)
  4. /markers/candidates + /markers/confirmed  — position-based ArUco mock
  5. /mission/takeoff_complete   — when altitude >= 1.9 m
  6. /mission/target_reached     — when dist to active target < 1.2 m
  7. /mission/vertiport_acquired — when dist to vertiport < 1.0 m  (RETURN_HOME)
  8. /mission/landing_complete   — when altitude <= 0.15 m
"""

import math
import sys
import threading

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from px4_msgs.msg import VehicleAttitude, BatteryStatus, TrajectorySetpoint
    from sprint_drone_msgs.msg import VelocityProfile, MarkerList, MarkerInfo
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import PoseStamped, Point
except ImportError as e:
    sys.exit(f"Import error: {e}\n  source /opt/ros/jazzy/setup.bash && source install/setup.bash")

# ── 마커 배치 (Gazebo world → grid frame) ─────────────────────────────────────
START_X, START_Y = 2.0, 19.0   # vertiport Gazebo 좌표

MARKERS = {
    1: {"gx": 19.0 - START_X, "gy": 19.0 - START_Y},   # grid (17, 0)
    2: {"gx":  4.0 - START_X, "gy": 16.0 - START_Y},   # grid (2, -3)
    3: {"gx": 10.0 - START_X, "gy":  4.0 - START_Y},   # grid (8, -15)
    4: {"gx": 25.0 - START_X, "gy":  4.0 - START_Y},   # grid (23, -15)
}

DETECT_RADIUS  = 1.8   # 마커 감지 반경 (m)
CONFIRM_RADIUS = 1.8   # confirmed 판정 반경 (실제 카메라는 ANTI_SWAY 중 호버에서 확인)
HOME_RADIUS    = 2.0   # vertiport 획득 반경 (WAYPOINT_TOL=1.5m 보다 크게)

PX4_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class MockSensors(Node):
    def __init__(self):
        super().__init__("mock_sensors")

        # ── 상태 ─────────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._gx = 0.0      # grid 좌표 (pose_grid 기반)
        self._gy = 0.0
        self._alt = 0.0     # 고도 (vehicle_local_position.z 부호 반전)
        self._mission_state = "INIT"
        self._prev_mission_state = "INIT"
        self._target_gx = 0.0
        self._target_gy = 0.0
        self._confirmed_ids: set = set()
        self._approaching_id: int | None = None  # 현재 mission pipeline이 처리 중인 마커 ID

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(PoseStamped, "/localization/pose_grid",
                                 self._on_pose_grid, 10)
        self.create_subscription(String, "/mission/state",
                                 self._on_state, 10)
        self.create_subscription(Point, "/mission/target_node",
                                 self._on_target, 10)
        self.create_subscription(VelocityProfile, "/planner/velocity_profile",
                                 self._on_profile, 10)

        from px4_msgs.msg import VehicleLocalPosition
        self.create_subscription(VehicleLocalPosition,
                                 "/fmu/out/vehicle_local_position",
                                 self._on_local_pos, PX4_QOS)

        # ── Publishers ────────────────────────────────────────────────────────
        RELIABLE_QOS = QoSProfile(depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        self._pub_att     = self.create_publisher(VehicleAttitude,
                                "/fmu/out/vehicle_attitude", PX4_QOS)
        self._pub_bat     = self.create_publisher(BatteryStatus,
                                "/fmu/out/battery_status", PX4_QOS)
        # gz_drone_sim subscribes with RELIABLE QoS (depth=10) — must match
        self._pub_traj    = self.create_publisher(TrajectorySetpoint,
                                "/fmu/in/trajectory_setpoint", RELIABLE_QOS)
        self._pub_cands   = self.create_publisher(MarkerList,
                                "/markers/candidates", 10)
        self._pub_conf    = self.create_publisher(MarkerList,
                                "/markers/confirmed", 10)
        self._pub_tko     = self.create_publisher(Bool,
                                "/mission/takeoff_complete", 10)
        self._pub_reached = self.create_publisher(Bool,
                                "/mission/target_reached", 10)
        self._pub_vp      = self.create_publisher(Bool,
                                "/vertiport/acquired", 10)
        self._pub_land    = self.create_publisher(Bool,
                                "/landing/complete", 10)

        # 50 Hz: attitude + trajectory + signal checks
        self.create_timer(0.02, self._fast_tick)
        # 10 Hz: battery + aruco
        self.create_timer(0.10, self._slow_tick)

        self.get_logger().info("mock_sensors: ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_pose_grid(self, msg: PoseStamped):
        with self._lock:
            self._gx = msg.pose.position.x
            self._gy = msg.pose.position.y

    def _on_state(self, msg: String):
        with self._lock:
            new_s = msg.data
            prev_s = self._prev_mission_state
            # HOVER_CONFIRM → MARKER_SAVE: 마커 저장 직전 → confirmed_ids 즉시 업데이트
            # (MARKER_SAVE 단계에서 slow_tick이 다시 candidate 발행하지 않도록)
            if new_s == "MARKER_SAVE" and prev_s == "HOVER_CONFIRM":
                if self._approaching_id is not None:
                    self._confirmed_ids.add(self._approaching_id)
            # MARKER_SAVE → GRID_SEARCH: approaching_id 초기화
            elif new_s == "GRID_SEARCH" and prev_s == "MARKER_SAVE":
                self._approaching_id = None
            # 다른 경로로 GRID_SEARCH 복귀: 접근 실패 → approaching_id만 초기화
            elif new_s == "GRID_SEARCH" and prev_s not in (
                    "", "INIT", "GRID_SEARCH", "HOME_INIT", "TAKEOFF"):
                self._approaching_id = None
            self._prev_mission_state = new_s
            self._mission_state = new_s

    def _on_target(self, msg: Point):
        with self._lock:
            self._target_gx = msg.x
            self._target_gy = msg.y

    def _on_local_pos(self, msg):
        with self._lock:
            self._alt = -msg.z   # NED z 부호 반전 → 고도

    def _on_profile(self, msg: VelocityProfile):
        """velocity_profile → TrajectorySetpoint (NED velocity)."""
        if not msg.waypoints or not msg.target_speeds_mps:
            return
        with self._lock:
            gx, gy = self._gx, self._gy
            state   = self._mission_state

        wp   = msg.waypoints[0]
        spd  = float(msg.target_speeds_mps[0])

        dx = wp.x - gx
        dy = wp.y - gy
        d  = math.sqrt(dx*dx + dy*dy)

        CRUISE_Z = 2.0
        with self._lock:
            alt = self._alt

        if state in ("INIT", "LANDED", "ABORT"):
            vn, ve, vz = 0.0, 0.0, 0.0
        elif state == "VISION_SERVO_LAND":
            # 착륙: 수평 정지, 0.5 m/s 하강 (NED: down=positive)
            vn, ve, vz = 0.0, 0.0, 0.5
        else:
            # 고도 제어: 순항 고도까지 상승/유지
            alt_err = CRUISE_Z - alt
            vz_ned  = -max(-1.7, min(1.7, alt_err * 1.5))   # NED: down=positive

            if d < 0.05 or spd < 0.01:
                vn, ve = 0.0, 0.0
            else:
                # grid → NED: north=grid_y, east=grid_x
                vn = (dy / d) * spd
                ve = (dx / d) * spd
            vz = vz_ned

        traj = TrajectorySetpoint()
        traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        traj.velocity   = [float(vn), float(ve), float(vz)]
        traj.position   = [float("nan")] * 3
        traj.yaw        = float("nan")
        self._pub_traj.publish(traj)

    # ── Fast tick (50 Hz) ─────────────────────────────────────────────────────

    def _fast_tick(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        # VehicleAttitude — identity quaternion (no yaw)
        att = VehicleAttitude()
        att.timestamp = now_us
        att.q = [1.0, 0.0, 0.0, 0.0]
        self._pub_att.publish(att)

        with self._lock:
            alt   = self._alt
            gx    = self._gx
            gy    = self._gy
            state = self._mission_state
            tgx   = self._target_gx
            tgy   = self._target_gy

        # takeoff_complete
        b = Bool(); b.data = (alt >= 1.9)
        self._pub_tko.publish(b)

        # target_reached
        dist_target = math.sqrt((gx - tgx)**2 + (gy - tgy)**2)
        b = Bool(); b.data = (dist_target < 1.2) and state not in ("INIT","TAKEOFF","HOME_INIT","LANDED")
        self._pub_reached.publish(b)

        # vertiport_acquired — when returning and near origin
        dist_home = math.sqrt(gx**2 + gy**2)
        b = Bool(); b.data = (dist_home < HOME_RADIUS) and state in (
            "RETURN_HOME", "VERTIPORT_ACQUIRE", "EMERGENCY_RETURN")
        self._pub_vp.publish(b)

        # landing_complete
        b = Bool(); b.data = (alt <= 0.15) and state == "VISION_SERVO_LAND"
        self._pub_land.publish(b)

    # ── Slow tick (10 Hz) ─────────────────────────────────────────────────────

    def _slow_tick(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        # BatteryStatus
        bat = BatteryStatus()
        bat.timestamp = now_us
        bat.remaining = 1.0
        self._pub_bat.publish(bat)

        # ArUco mock detection
        with self._lock:
            gx    = self._gx
            gy    = self._gy
            state = self._mission_state

        if state in ("INIT", "LANDED", "ABORT"):
            return

        stamp = self.get_clock().now().to_msg()
        cands_msg = MarkerList()
        cands_msg.header.stamp = stamp
        conf_msg  = MarkerList()
        conf_msg.header.stamp  = stamp

        for mid, mpos in MARKERS.items():
            d = math.sqrt((gx - mpos["gx"])**2 + (gy - mpos["gy"])**2)

            # confirmed된 마커는 candidate 재발행 안 함 (GRID_SEARCH 재진입 방지)
            # confirmed_ids는 MARKER_SAVE 완료 시 _on_state에서만 추가됨
            if d < DETECT_RADIUS and mid not in self._confirmed_ids:
                m = MarkerInfo()
                m.header.stamp = stamp
                m.id = mid
                m.grid_position.x = mpos["gx"]
                m.grid_position.y = mpos["gy"]
                m.grid_position.z = 0.0
                m.confidence = max(0.0, 1.0 - d / DETECT_RADIUS)
                m.status = "CANDIDATE"
                cands_msg.markers.append(m)
                # 처음 candidate를 발행할 때 어떤 마커를 추적 중인지 기록
                if self._approaching_id is None:
                    self._approaching_id = mid

            # confirmed는 범위 내에 있는 동안 계속 발행 (ANTI_SWAY → HoverConfirm 전환 유지)
            if d < CONFIRM_RADIUS:
                m2 = MarkerInfo()
                m2.header.stamp = stamp
                m2.id = mid
                m2.grid_position.x = mpos["gx"]
                m2.grid_position.y = mpos["gy"]
                m2.grid_position.z = 0.0
                m2.confidence = 1.0
                m2.status = "CONFIRMED"
                conf_msg.markers.append(m2)

        self._pub_cands.publish(cands_msg)
        self._pub_conf.publish(conf_msg)


def main():
    rclpy.init()
    node = MockSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
