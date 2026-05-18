#!/usr/bin/env python3
"""
log_analyzer.py — rosbag2 mission data analysis.

Usage:
    python3 tools/log_analyzer.py <bag_path> [--verbose]

Reads a rosbag2 and reports:
  - Total mission time
  - Time spent in each mission state / controller mode
  - Marker detection accuracy (confirmed vs total candidates)
  - Battery consumption (start %, end %, draw rate)
  - Average sway metric per phase
  - Localization drift statistics

Requires:
    pip install rosbags         # pure-Python rosbag2 reader (no ROS2 install needed)
    pip install pandas numpy
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
except ImportError:
    print("ERROR: Install rosbags with:  pip install rosbags", file=sys.stderr)
    sys.exit(1)

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False


def ns_to_sec(ns: int) -> float:
    return ns * 1e-9


def msg_time(msg) -> float:
    """Extract timestamp in seconds from a stamped message."""
    if hasattr(msg, "header"):
        return ns_to_sec(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
    if hasattr(msg, "timestamp"):
        return msg.timestamp * 1e-6  # px4 uses microseconds
    return 0.0


# ── Phase timing ──────────────────────────────────────────────────────────────

class PhaseClock:
    """Accumulate wall time per labeled phase."""

    def __init__(self):
        self.totals: dict[str, float] = defaultdict(float)
        self._current: str | None = None
        self._entry_time: float = 0.0

    def transition(self, new_phase: str, t: float):
        if self._current is not None:
            self.totals[self._current] += t - self._entry_time
        self._current = new_phase
        self._entry_time = t

    def close(self, t: float):
        if self._current is not None:
            self.totals[self._current] += t - self._entry_time
            self._current = None


# ── Bag reader ────────────────────────────────────────────────────────────────

TOPICS = {
    "/mission/state":       "std_msgs/msg/String",
    "/controller/mode":     "std_msgs/msg/String",
    "/battery/percent":     "std_msgs/msg/Float32",
    "/sway/metric":         "std_msgs/msg/Float32",
    "/markers/candidates":  "sprint_drone_msgs/msg/MarkerList",
    "/localization/pose_grid": "geometry_msgs/msg/PoseStamped",
}


def load_typestore(repo_root: Path):
    store = get_typestore(Stores.ROS2_HUMBLE)  # compatible with Jazzy messages
    msg_dir = repo_root / "src" / "sprint_drone_msgs" / "msg"
    if msg_dir.exists():
        custom_types = {}
        for msg_file in sorted(msg_dir.glob("*.msg")):
            typename = f"sprint_drone_msgs/msg/{msg_file.stem}"
            custom_types.update(get_types_from_msg(msg_file.read_text(), typename))
        if custom_types:
            store.register(custom_types)
    return store


def analyze(bag_path: Path, verbose: bool = False, repo_root: Path | None = None) -> dict:
    repo_root = repo_root or Path(__file__).resolve().parents[1]
    store = load_typestore(repo_root)

    mission_clock = PhaseClock()
    ctrl_clock    = PhaseClock()

    battery_samples:    list[tuple[float, float]] = []  # (t, %)
    sway_per_state:     dict[str, list[float]]    = defaultdict(list)
    current_mission_state = "UNKNOWN"

    marker_candidate_counts: list[int] = []
    confirmed_markers:       set[int]  = set()

    pose_samples: list[tuple[float, float, float]] = []  # (t, x, y)

    first_t = last_t = None

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic in TOPICS]
        if not connections:
            print(f"WARNING: none of the expected topics found in {bag_path}")

        for conn, timestamp, raw in reader.messages(connections=connections):
            msg = store.deserialize_ros1(raw, conn.msgtype) if False \
                  else store.deserialize_cdr(raw, conn.msgtype)
            t = ns_to_sec(timestamp)

            if first_t is None:
                first_t = t
            last_t = t

            topic = conn.topic

            if topic == "/mission/state":
                new_state = msg.data
                if new_state != current_mission_state:
                    mission_clock.transition(new_state, t)
                    if verbose:
                        print(f"  [{t - first_t:7.2f}s] mission → {new_state}")
                current_mission_state = new_state

            elif topic == "/controller/mode":
                ctrl_clock.transition(msg.data, t)

            elif topic == "/battery/percent":
                battery_samples.append((t, msg.data))

            elif topic == "/sway/metric":
                sway_per_state[current_mission_state].append(msg.data)

            elif topic == "/markers/candidates":
                marker_candidate_counts.append(len(msg.markers))
                for m in msg.markers:
                    if hasattr(m, "id") and hasattr(m, "confidence"):
                        if m.confidence >= 0.75:
                            confirmed_markers.add(m.id)

            elif topic == "/localization/pose_grid":
                pose_samples.append((t, msg.pose.position.x, msg.pose.position.y))

    if first_t is None:
        print("ERROR: bag is empty or has no matching topics.", file=sys.stderr)
        sys.exit(1)

    mission_clock.close(last_t)
    ctrl_clock.close(last_t)

    total_time = last_t - first_t

    # Battery
    batt_start = battery_samples[0][1]  if battery_samples else float("nan")
    batt_end   = battery_samples[-1][1] if battery_samples else float("nan")
    batt_draw  = (batt_start - batt_end) / total_time * 60 if total_time > 0 else float("nan")

    # Sway per state
    sway_means = {
        state: float(np.mean(vals)) if vals else 0.0
        for state, vals in sway_per_state.items()
    }

    # Localization: compute max drift step
    drift_steps: list[float] = []
    for i in range(1, len(pose_samples)):
        dx = pose_samples[i][1] - pose_samples[i-1][1]
        dy = pose_samples[i][2] - pose_samples[i-1][2]
        drift_steps.append(float(np.sqrt(dx**2 + dy**2)))
    max_drift_step = max(drift_steps) if drift_steps else 0.0

    result = {
        "total_time_sec": total_time,
        "mission_phase_times": dict(mission_clock.totals),
        "controller_mode_times": dict(ctrl_clock.totals),
        "battery_start_pct": batt_start,
        "battery_end_pct": batt_end,
        "battery_draw_pct_per_min": batt_draw,
        "sway_mean_per_state": sway_means,
        "confirmed_marker_ids": sorted(confirmed_markers),
        "confirmed_marker_count": len(confirmed_markers),
        "localization_max_step_m": max_drift_step,
        "localization_sample_count": len(pose_samples),
    }
    return result


def print_report(r: dict):
    print("=" * 60)
    print(f"  Mission Analysis Report")
    print("=" * 60)
    print(f"  Total mission time  : {r['total_time_sec']:.1f} s  ({r['total_time_sec']/60:.1f} min)")
    print()

    print("  Mission state times (s):")
    for state, t in sorted(r["mission_phase_times"].items(), key=lambda x: -x[1]):
        print(f"    {state:<25} {t:6.1f} s  ({t/r['total_time_sec']*100:.1f}%)")
    print()

    print("  Controller mode times (s):")
    for mode, t in sorted(r["controller_mode_times"].items(), key=lambda x: -x[1]):
        print(f"    {mode:<25} {t:6.1f} s")
    print()

    print("  Battery:")
    print(f"    Start               : {r['battery_start_pct']:.1f} %")
    print(f"    End                 : {r['battery_end_pct']:.1f} %")
    print(f"    Draw rate           : {r['battery_draw_pct_per_min']:.2f} %/min")
    print()

    print("  Sway (mean rad) per state:")
    for state, val in sorted(r["sway_mean_per_state"].items()):
        print(f"    {state:<25} {val:.4f} rad")
    print()

    print(f"  Confirmed markers     : {r['confirmed_marker_count']}/4  IDs={r['confirmed_marker_ids']}")
    print()

    print(f"  Localization:")
    print(f"    Max step jump       : {r['localization_max_step_m']:.3f} m")
    print(f"    Sample count        : {r['localization_sample_count']}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Analyze a drone mission rosbag2")
    parser.add_argument("bag", type=Path, help="Path to rosbag2 directory or .db3 file")
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1],
                        help="Repository root for custom message definitions")
    parser.add_argument("--verbose", action="store_true", help="Print state transitions as they occur")
    parser.add_argument("--json", action="store_true", help="Output raw JSON instead of formatted report")
    args = parser.parse_args()

    if not args.bag.exists():
        print(f"ERROR: {args.bag} does not exist", file=sys.stderr)
        sys.exit(1)

    result = analyze(args.bag, verbose=args.verbose, repo_root=args.repo_root)

    if args.json:
        import json
        print(json.dumps(result, indent=2, default=str))
    else:
        print_report(result)


if __name__ == "__main__":
    main()
