#!/usr/bin/env python3
"""Plot a sprint-drone rosbag2 mission run.

The plot overlays the estimated grid-frame trajectory, confirmed marker
positions, target points, and mission-state transitions on the contest field.

Usage:
    python3 tools/plot_mission.py <bag_path>
    python3 tools/plot_mission.py <bag_path> --output reports/run_01.png
    python3 tools/plot_mission.py <bag_path> --show

Dependencies:
    python3 -m venv .venv
    . .venv/bin/activate
    pip install rosbags matplotlib numpy
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


POSE_TOPIC = "/localization/pose_grid"
CONFIRMED_TOPIC = "/markers/confirmed"
CANDIDATE_TOPIC = "/markers/candidates"
MISSION_STATE_TOPIC = "/mission/state"
CONTROLLER_MODE_TOPIC = "/controller/mode"
MISSION_TARGET_TOPIC = "/mission/target_node"
PLANNER_TOPIC = "/planner/velocity_profile"

TOPICS = {
    POSE_TOPIC,
    CONFIRMED_TOPIC,
    CANDIDATE_TOPIC,
    MISSION_STATE_TOPIC,
    CONTROLLER_MODE_TOPIC,
    MISSION_TARGET_TOPIC,
    PLANNER_TOPIC,
}


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    z: float


@dataclass
class MarkerSample:
    t: float
    marker_id: int
    x: float
    y: float
    confidence: float
    status: str = ""


@dataclass
class LabelSample:
    t: float
    label: str


@dataclass
class PointSample:
    t: float
    x: float
    y: float


@dataclass
class MissionData:
    poses: list[PoseSample] = field(default_factory=list)
    confirmed_markers: list[MarkerSample] = field(default_factory=list)
    candidate_markers: list[MarkerSample] = field(default_factory=list)
    mission_states: list[LabelSample] = field(default_factory=list)
    controller_modes: list[LabelSample] = field(default_factory=list)
    mission_targets: list[PointSample] = field(default_factory=list)
    planner_targets: list[PointSample] = field(default_factory=list)
    first_t: float | None = None
    last_t: float | None = None


def load_typestore(repo_root: Path) -> Any:
    try:
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ImportError:
        sys.exit(
            "Missing dependency: rosbags\n"
            "Create a venv and install tools deps:\n"
            "  python3 -m venv .venv\n"
            "  . .venv/bin/activate\n"
            "  pip install rosbags matplotlib numpy"
        )

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    msg_dir = repo_root / "src" / "sprint_drone_msgs" / "msg"
    if msg_dir.exists():
        custom_types: dict[str, Any] = {}
        for msg_file in sorted(msg_dir.glob("*.msg")):
            typename = f"sprint_drone_msgs/msg/{msg_file.stem}"
            custom_types.update(get_types_from_msg(msg_file.read_text(), typename))
        if custom_types:
            typestore.register(custom_types)
    return typestore


def msg_timestamp_sec(msg: Any, fallback_sec: float) -> float:
    if hasattr(msg, "header"):
        stamp = msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if hasattr(msg, "timestamp"):
        return float(msg.timestamp) * 1e-6
    return fallback_sec


def normalize_time(data: MissionData) -> None:
    if data.first_t is None:
        return
    t0 = data.first_t
    for items in (
        data.poses,
        data.confirmed_markers,
        data.candidate_markers,
        data.mission_states,
        data.controller_modes,
        data.mission_targets,
        data.planner_targets,
    ):
        for item in items:
            item.t -= t0
    data.last_t = None if data.last_t is None else data.last_t - t0
    data.first_t = 0.0


def append_markers(target: list[MarkerSample], msg: Any, t: float) -> None:
    for marker in getattr(msg, "markers", []):
        pos = getattr(marker, "grid_position", None)
        if pos is None:
            continue
        target.append(
            MarkerSample(
                t=t,
                marker_id=int(marker.id),
                x=float(pos.x),
                y=float(pos.y),
                confidence=float(getattr(marker, "confidence", 0.0)),
                status=str(getattr(marker, "status", "")),
            )
        )


def read_bag(bag_path: Path, repo_root: Path) -> MissionData:
    try:
        from rosbags.rosbag2 import Reader
    except ImportError:
        sys.exit(
            "Missing dependency: rosbags\n"
            "Create a venv and install tools deps:\n"
            "  python3 -m venv .venv\n"
            "  . .venv/bin/activate\n"
            "  pip install rosbags matplotlib numpy"
        )

    typestore = load_typestore(repo_root)
    data = MissionData()

    with Reader(bag_path) as reader:
        connections = [conn for conn in reader.connections if conn.topic in TOPICS]
        if not connections:
            found = ", ".join(sorted(conn.topic for conn in reader.connections))
            sys.exit(f"No expected topics found in {bag_path}. Found: {found}")

        for conn, timestamp, raw in reader.messages(connections=connections):
            fallback_t = float(timestamp) * 1e-9
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            t = msg_timestamp_sec(msg, fallback_t)
            if t <= 0.0:
                t = fallback_t

            if data.first_t is None:
                data.first_t = t
            data.last_t = t

            if conn.topic == POSE_TOPIC:
                data.poses.append(
                    PoseSample(
                        t=t,
                        x=float(msg.pose.position.x),
                        y=float(msg.pose.position.y),
                        z=float(msg.pose.position.z),
                    )
                )
            elif conn.topic == CONFIRMED_TOPIC:
                append_markers(data.confirmed_markers, msg, t)
            elif conn.topic == CANDIDATE_TOPIC:
                append_markers(data.candidate_markers, msg, t)
            elif conn.topic == MISSION_STATE_TOPIC:
                label = str(msg.data)
                if not data.mission_states or data.mission_states[-1].label != label:
                    data.mission_states.append(LabelSample(t=t, label=label))
            elif conn.topic == CONTROLLER_MODE_TOPIC:
                label = str(msg.data)
                if not data.controller_modes or data.controller_modes[-1].label != label:
                    data.controller_modes.append(LabelSample(t=t, label=label))
            elif conn.topic == MISSION_TARGET_TOPIC:
                data.mission_targets.append(PointSample(t=t, x=float(msg.x), y=float(msg.y)))
            elif conn.topic == PLANNER_TOPIC:
                if getattr(msg, "waypoints", []):
                    wp = msg.waypoints[0]
                    data.planner_targets.append(PointSample(t=t, x=float(wp.x), y=float(wp.y)))

    normalize_time(data)
    return data


def last_marker_by_id(markers: list[MarkerSample]) -> dict[int, MarkerSample]:
    out: dict[int, MarkerSample] = {}
    for marker in markers:
        out[marker.marker_id] = marker
    return out


def nearest_pose(poses: list[PoseSample], t: float) -> PoseSample | None:
    if not poses:
        return None
    return min(poses, key=lambda pose: abs(pose.t - t))


def maybe_import_matplotlib(show: bool) -> Any:
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        sys.exit(
            "Missing dependency: matplotlib\n"
            "Create a venv and install tools deps:\n"
            "  python3 -m venv .venv\n"
            "  . .venv/bin/activate\n"
            "  pip install rosbags matplotlib numpy"
        )
    return plt, Rectangle


def plot_field(ax: Any, rectangle_cls: Any) -> None:
    ax.add_patch(rectangle_cls((0, 0), 32, 23, fill=False, edgecolor="#667085", linewidth=1.5))
    ax.add_patch(rectangle_cls((4, 4), 24, 15, fill=False, edgecolor="#98a2b3", linewidth=1.2))

    for x in range(4, 29, 3):
        ax.plot([x, x], [4, 19], color="#d0d5dd", linewidth=0.6, zorder=0)
    for y in range(4, 20, 3):
        ax.plot([4, 28], [y, y], color="#d0d5dd", linewidth=0.6, zorder=0)

    ax.scatter([2], [19], s=500, color="#d77b18", edgecolor="#7a3f00", zorder=2)
    ax.text(2, 20.8, "Vertiport", ha="center", va="bottom", fontsize=8)
    ax.text(4, 3.4, "Mission area 24x15m", fontsize=8, color="#475467")
    ax.text(0, -0.7, "Safe area 32x23m", fontsize=8, color="#475467")


def plot_mission(data: MissionData, output: Path, show: bool) -> None:
    plt, rectangle_cls = maybe_import_matplotlib(show)
    fig, ax = plt.subplots(figsize=(12, 8))
    plot_field(ax, rectangle_cls)

    if data.poses:
        xs = [p.x for p in data.poses]
        ys = [p.y for p in data.poses]
        ax.plot(xs, ys, color="#2563eb", linewidth=2.0, label="pose_grid path", zorder=3)
        ax.scatter(xs[:1], ys[:1], s=70, marker="o", color="#16a34a", label="start", zorder=4)
        ax.scatter(xs[-1:], ys[-1:], s=70, marker="X", color="#dc2626", label="end", zorder=4)

    confirmed = last_marker_by_id(data.confirmed_markers)
    if confirmed:
        mx = [m.x for m in confirmed.values()]
        my = [m.y for m in confirmed.values()]
        ax.scatter(mx, my, s=120, marker="s", color="#12b76a", edgecolor="#054f31",
                   label="confirmed markers", zorder=5)
        for marker in confirmed.values():
            ax.text(marker.x + 0.25, marker.y + 0.25, f"ID {marker.marker_id}", fontsize=9)

    candidates = last_marker_by_id(data.candidate_markers)
    if candidates:
        cx = [m.x for m in candidates.values()]
        cy = [m.y for m in candidates.values()]
        ax.scatter(cx, cy, s=70, marker="s", facecolors="none", edgecolors="#17b26a",
                   alpha=0.45, label="candidate markers", zorder=4)

    if data.mission_targets:
        tx = [p.x for p in data.mission_targets]
        ty = [p.y for p in data.mission_targets]
        ax.scatter(tx, ty, s=30, marker="+", color="#7c3aed", label="mission targets", zorder=4)

    for state in data.mission_states:
        pose = nearest_pose(data.poses, state.t)
        if pose is None:
            continue
        ax.scatter([pose.x], [pose.y], s=35, color="#f04438", zorder=6)
        ax.text(pose.x + 0.18, pose.y - 0.18, state.label, fontsize=7, color="#7a271a")

    total = data.last_t if data.last_t is not None else 0.0
    ax.set_title(f"Sprint Drone Mission Plot ({total:.1f}s)")
    ax.set_xlabel("Grid/world x (m)")
    ax.set_ylabel("Grid/world y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1, 33)
    ax.set_ylim(23.5, -1.5)
    ax.grid(False)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    if show:
        plt.show()
    plt.close(fig)


def default_output_path(bag_path: Path) -> Path:
    if bag_path.is_dir():
        return bag_path / "mission_plot.png"
    return bag_path.with_suffix(".mission_plot.png")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a sprint drone mission rosbag2")
    parser.add_argument("bag", type=Path, help="rosbag2 directory or .db3 path")
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1],
                        help="Repository root for custom message definitions")
    parser.add_argument("--output", type=Path, default=None,
                        help="Output PNG path (default: <bag>/mission_plot.png)")
    parser.add_argument("--show", action="store_true", help="Show the plot interactively")
    args = parser.parse_args()

    if not args.bag.exists():
        sys.exit(f"Bag path does not exist: {args.bag}")

    output = args.output or default_output_path(args.bag)
    data = read_bag(args.bag, args.repo_root)
    plot_mission(data, output, args.show)

    print(f"Wrote mission plot: {output}")
    print(f"  poses: {len(data.poses)}")
    print(f"  confirmed marker ids: {sorted(last_marker_by_id(data.confirmed_markers))}")
    print(f"  mission transitions: {len(data.mission_states)}")


if __name__ == "__main__":
    main()
