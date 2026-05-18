#!/usr/bin/env python3
"""Generate report figures from the existing Gazebo/ROS2 mission stack.

The intended input is a rosbag captured by scripts/sim_run.sh, which runs the
team's Gazebo competition world, ROS-GZ bridge, perception/planner/mission/
control launch files, and standalone camera-rig motion simulator.

Usage:
    python3 reports/generate_report_figures.py bags/run_standalone_42_YYYYMMDD_HHMMSS
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import FancyBboxPatch, Rectangle
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


POSE_TOPIC = "/localization/pose_grid"
STATE_TOPIC = "/mission/state"
MODE_TOPIC = "/controller/mode"
SETPOINT_TOPIC = "/fmu/in/trajectory_setpoint"
LOCAL_POS_TOPIC = "/fmu/out/vehicle_local_position"
IMAGE_TOPIC = "/drone/camera/down/image_raw"
GRID_TOPIC = "/grid/intersections"
MARKER_TOPIC = "/markers/confirmed"
PLANNER_TOPIC = "/planner/velocity_profile"
MARKERS_GRID = {
    1: (17.0, 0.0),
    2: (2.0, -3.0),
    3: (8.0, -15.0),
    4: (23.0, -15.0),
}

KO_FONT_PATH = Path("/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf")
if KO_FONT_PATH.exists():
    fm.fontManager.addfont(str(KO_FONT_PATH))
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=str(KO_FONT_PATH)).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False


@dataclass
class Sample:
    t: float
    value: Any


@dataclass
class RunData:
    poses: list[tuple[float, float, float, float]] = field(default_factory=list)
    states: list[Sample] = field(default_factory=list)
    modes: list[Sample] = field(default_factory=list)
    speeds: list[tuple[float, float]] = field(default_factory=list)
    planner_speeds: list[tuple[float, float]] = field(default_factory=list)
    altitudes: list[tuple[float, float]] = field(default_factory=list)
    first_image: Any | None = None
    first_image_t: float | None = None
    best_image: Any | None = None
    best_image_t: float | None = None
    best_image_score: float = -1.0
    topic_times: dict[str, list[float]] = field(default_factory=dict)
    image_count: int = 0
    grid_count: int = 0
    marker_count: int = 0
    planner_count: int = 0
    first_t: float | None = None
    last_t: float | None = None


def load_typestore(repo_root: Path):
    store = get_typestore(Stores.ROS2_JAZZY)
    custom_types: dict[str, Any] = {}
    for package in ("sprint_drone_msgs", "px4_msgs"):
        msg_dir = repo_root / "src" / package / "msg"
        if not msg_dir.exists():
            continue
        for msg_file in sorted(msg_dir.glob("*.msg")):
            typename = f"{package}/msg/{msg_file.stem}"
            custom_types.update(get_types_from_msg(msg_file.read_text(), typename))
    if custom_types:
        store.register(custom_types)
    return store


def stamp_sec(msg: Any, fallback_ns: int) -> float:
    if hasattr(msg, "header"):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    if hasattr(msg, "timestamp"):
        return float(msg.timestamp) * 1e-6
    return float(fallback_ns) * 1e-9


def append_transition(items: list[Sample], t: float, label: str) -> None:
    if not items or items[-1].value != label:
        items.append(Sample(t, label))


def decode_image(msg: Any) -> np.ndarray | None:
    if msg.encoding not in ("rgb8", "bgr8"):
        return None
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        int(msg.height), int(msg.width), 3)
    if msg.encoding == "bgr8":
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
    return arr


def read_bag(bag_path: Path, repo_root: Path) -> RunData:
    store = load_typestore(repo_root)
    wanted = {
        POSE_TOPIC,
        STATE_TOPIC,
        MODE_TOPIC,
        SETPOINT_TOPIC,
        LOCAL_POS_TOPIC,
        IMAGE_TOPIC,
        GRID_TOPIC,
        MARKER_TOPIC,
        PLANNER_TOPIC,
    }
    data = RunData()
    with Reader(bag_path) as reader:
        conns = [conn for conn in reader.connections if conn.topic in wanted]
        for conn, ts, raw in reader.messages(connections=conns):
            msg = store.deserialize_cdr(raw, conn.msgtype)
            # Use the rosbag receive timestamp for all topics. Some PX4-style
            # messages carry microsecond clocks while ROS messages use header
            # stamps; mixing them makes relative times meaningless.
            t = float(ts) * 1e-9
            if data.first_t is None:
                data.first_t = t
            data.last_t = t

            rel_t = t - data.first_t
            data.topic_times.setdefault(conn.topic, []).append(rel_t)
            if conn.topic == POSE_TOPIC:
                data.poses.append((
                    rel_t,
                    float(msg.pose.position.x),
                    float(msg.pose.position.y),
                    float(msg.pose.position.z),
                ))
            elif conn.topic == STATE_TOPIC:
                append_transition(data.states, rel_t, str(msg.data))
            elif conn.topic == MODE_TOPIC:
                append_transition(data.modes, rel_t, str(msg.data))
            elif conn.topic == SETPOINT_TOPIC:
                vx, vy, vz = [0.0 if math.isnan(float(v)) else float(v) for v in msg.velocity]
                data.speeds.append((rel_t, math.sqrt(vx * vx + vy * vy + vz * vz)))
            elif conn.topic == LOCAL_POS_TOPIC:
                data.altitudes.append((rel_t, -float(msg.z)))
            elif conn.topic == IMAGE_TOPIC:
                data.image_count += 1
                image = decode_image(msg)
                if data.first_image is None:
                    data.first_image = image
                    data.first_image_t = rel_t
                if image is not None:
                    score = float(np.std(image)) + 0.01 * float(np.mean(image))
                    if score > data.best_image_score:
                        data.best_image = image
                        data.best_image_t = rel_t
                        data.best_image_score = score
            elif conn.topic == GRID_TOPIC:
                data.grid_count += 1
            elif conn.topic == MARKER_TOPIC:
                data.marker_count += 1
            elif conn.topic == PLANNER_TOPIC:
                data.planner_count += 1
                if msg.target_speeds_mps:
                    data.planner_speeds.append((rel_t, float(msg.target_speeds_mps[0])))
    return data


FIG_COLORS = {
    "blue": "#2166ac",
    "orange": "#e08214",
    "green": "#1b7837",
    "red": "#b2182b",
    "purple": "#762a83",
    "slate": "#344054",
    "grid": "#d0d5dd",
    "panel": "#f8fafc",
}


plt.rcParams.update({
    "axes.linewidth": 0.8,
    "axes.titlesize": 9.5,
    "axes.labelsize": 8.5,
    "xtick.labelsize": 7.5,
    "ytick.labelsize": 7.5,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "xtick.top": True,
    "ytick.right": True,
    "legend.fontsize": 7.5,
    "legend.frameon": True,
    "legend.fancybox": False,
})


def style_axes(ax):
    ax.grid(True, color=FIG_COLORS["grid"], linewidth=0.55, alpha=0.7)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_linewidth(0.8)


def save_system_block(out: Path) -> None:
    fig, ax = plt.subplots(figsize=(10.8, 5.6), dpi=220)
    ax.axis("off")
    ax.set_xlim(0, 10.8)
    ax.set_ylim(0, 5.6)

    def box(x, y, w, h, title, body, color):
        patch = FancyBboxPatch(
            (x, y), w, h,
            boxstyle="round,pad=0.025,rounding_size=0.045",
            facecolor=color,
            edgecolor="#475467",
            linewidth=0.95,
        )
        ax.add_patch(patch)
        ax.text(x + w / 2, y + h * 0.64, title, fontsize=9.2, weight="bold",
                color="#101828", ha="center", va="center")
        ax.text(x + w / 2, y + h * 0.30, body, fontsize=7.4, color="#344054",
                ha="center", va="center")

    ax.text(5.4, 5.28, "GPS-Free 고속 자율비행 시스템 구성",
            ha="center", va="center", fontsize=10.5, weight="bold", color="#101828")

    # Physical sensors and aircraft stack.
    box(0.35, 3.75, 1.7, 0.88, "Camera", "downward\n60 fps target", "#e0f2fe")
    box(0.35, 2.65, 1.7, 0.88, "IMU / Baro", "attitude\naltitude cue", "#f8fafc")
    box(0.35, 1.55, 1.7, 0.88, "ToF Range", "2 m scale\nheight gate", "#f8fafc")
    box(0.35, 0.45, 1.7, 0.88, "C2 / Ground", "start / abort\ntelemetry", "#fee2e2")

    # Team-owned software stack.
    sw = Rectangle((2.45, 0.28), 4.35, 4.52, facecolor="none",
                   edgecolor="#667085", linewidth=0.9, linestyle="--")
    ax.add_patch(sw)
    ax.text(4.62, 4.98, "Team-owned ROS2 C++ software", ha="center",
            fontsize=8.2, color="#344054")
    box(2.75, 3.75, 1.55, 0.78, "Perception", "grid / ArUco\nvertiport", "#dcfce7")
    box(4.8, 3.75, 1.55, 0.78, "Localization", "optical flow\nnode snap", "#dbeafe")
    box(2.75, 2.45, 1.55, 0.78, "Mission", "phase logic\nmarker memory", "#fef3c7")
    box(4.8, 2.45, 1.55, 0.78, "Planner", "serpentine\nsprint route", "#f3e8ff")
    box(3.78, 1.15, 1.55, 0.78, "Offboard", "velocity / yaw\nsetpoints", "#dcfce7")

    box(7.35, 2.95, 1.55, 0.95, "PX4 FC", "attitude control\nfailsafe", "#fff7ed")
    box(9.15, 2.95, 1.25, 0.95, "ESC / Motor", "propulsion", "#fce7f3")
    box(7.35, 1.25, 1.55, 0.95, "Power", "battery\n5 V DC-DC", "#f8fafc")

    def arrow(a, b, text=None, dy=0.0):
        ax.annotate("", xy=b, xytext=a,
                    arrowprops=dict(arrowstyle="-|>", color="#475467", linewidth=1.15,
                                    shrinkA=3, shrinkB=3))
        if text:
            ax.text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + dy, text,
                    fontsize=6.8, color="#344054", ha="center",
                    bbox=dict(boxstyle="round,pad=0.08", fc="white", ec="none", alpha=0.9))

    def poly_arrow(points, text=None, text_xy=None):
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        ax.plot(xs[:-1], ys[:-1], color="#475467", linewidth=1.15)
        ax.annotate("", xy=points[-1], xytext=points[-2],
                    arrowprops=dict(arrowstyle="-|>", color="#475467", linewidth=1.15,
                                    shrinkA=0, shrinkB=3))
        if text and text_xy:
            ax.text(text_xy[0], text_xy[1], text, fontsize=6.8, color="#344054",
                    ha="center",
                    bbox=dict(boxstyle="round,pad=0.08", fc="white", ec="none", alpha=0.9))

    arrow((2.05, 4.19), (2.75, 4.19), "image")
    poly_arrow([(2.05, 3.09), (2.22, 3.09), (2.22, 4.68), (5.20, 4.68), (5.20, 4.53)],
               "attitude", (3.28, 4.58))
    poly_arrow([(2.05, 1.99), (2.38, 1.99), (2.38, 4.83), (5.95, 4.83), (5.95, 4.53)],
               "height", (4.12, 4.73))
    arrow((2.05, 0.89), (2.75, 2.68), "mission cmd")
    arrow((4.3, 4.14), (4.8, 4.14))
    arrow((5.58, 3.75), (5.58, 3.23), "pose", dy=0.0)
    arrow((4.3, 2.84), (4.8, 2.84))
    arrow((5.58, 2.45), (4.55, 1.93), "target")
    arrow((5.33, 1.54), (7.35, 3.35), "/fmu/in/*")
    arrow((8.9, 3.43), (9.15, 3.43))
    arrow((8.13, 2.2), (8.13, 2.95), "power")

    ax.text(5.4, 0.08,
            "Key emphasis: visual navigation, grid-snap localization, candidate detection, and direct rescue sprint",
            fontsize=7.8, color="#344054", ha="center")
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def serpentine_points() -> np.ndarray:
    pts = []
    for row in range(6):
        y = -row * 3.0
        cols = range(9) if row % 2 == 0 else range(8, -1, -1)
        for col in cols:
            pts.append((2.0 + col * 3.0, y))
    return np.asarray(pts, dtype=float)


def plot_grid_field(ax):
    ax.add_patch(Rectangle((2, -15), 24, 15, fill=False, edgecolor="#667085", linewidth=1.5))
    for x in np.arange(2, 26.1, 3):
        ax.plot([x, x], [-15, 0], color="#d0d5dd", linewidth=0.8, zorder=0)
    for y in np.arange(-15, 0.1, 3):
        ax.plot([2, 26], [y, y], color="#d0d5dd", linewidth=0.8, zorder=0)
    ax.scatter([0], [0], marker="H", s=90, color="#f97316", edgecolor="#111827", zorder=5)
    ax.text(0, 0.75, "Home", ha="center", fontsize=7.5)


def plot_marker_layout(ax):
    for mid, (mx, my) in MARKERS_GRID.items():
        ax.scatter([mx], [my], marker="s", s=95, color="#e08214",
                   edgecolor="white", linewidth=0.8, zorder=6)
        ax.text(mx, my + 0.75, f"M{mid}", ha="center", va="bottom",
                fontsize=8, weight="bold", color="#9a3412",
                bbox=dict(boxstyle="round,pad=0.12", facecolor="white",
                          edgecolor="none", alpha=0.82), zorder=7)


def save_search_path(data: RunData, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(11, 6), dpi=180)
    plot_grid_field(ax)
    plot_marker_layout(ax)
    pts = serpentine_points()
    ax.plot(pts[:, 0], pts[:, 1], "--", color="#98a2b3", linewidth=1.4,
            label="planned 54-intersection serpentine")
    ax.scatter(pts[:, 0], pts[:, 1], s=16, color="#475467", zorder=3)
    if data.poses:
        pose = np.asarray(data.poses)
        ax.plot(pose[:, 1], pose[:, 2], color="#2563eb", linewidth=2.0,
                label=f"Gazebo-recorded trajectory ({len(data.poses)} samples)")
        ax.scatter(pose[0, 1], pose[0, 2], s=60, color="#16a34a", zorder=5, label="record start")
        ax.scatter(pose[-1, 1], pose[-1, 2], s=60, color="#dc2626", zorder=5, label="record end")
    ax.set_title("Phase 1 Search Path from Gazebo Run")
    ax.set_xlabel("Grid X / east from vertiport (m)")
    ax.set_ylabel("Grid Y / north from vertiport (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1, 28)
    ax.set_ylim(-17, 2)
    style_axes(ax)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)


def save_mission_path(data: RunData, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(11, 6), dpi=180)
    plot_grid_field(ax)
    plot_marker_layout(ax)
    if data.poses:
        pose = np.asarray(data.poses)
        ax.plot(pose[:, 1], pose[:, 2], color="#1d4ed8", linewidth=2.0,
                label="Gazebo-recorded mission path")
        ax.scatter(pose[0, 1], pose[0, 2], s=65, color="#16a34a", zorder=5, label="start")
        ax.scatter(pose[-1, 1], pose[-1, 2], s=65, color="#dc2626", zorder=5, label="end")
    ax.set_title("Gazebo Mission Path Overview")
    ax.set_xlabel("Grid X / east from vertiport (m)")
    ax.set_ylabel("Grid Y / north from vertiport (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1, 28)
    ax.set_ylim(-17, 2)
    style_axes(ax)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)


_PHASE_COLORS = {
    "INIT": "#f1f5f9", "TAKEOFF": "#dbeafe", "HOME_INIT": "#bfdbfe",
    "GRID_SEARCH": "#dcfce7", "MARKER_APPROACH": "#fef9c3",
    "ANTI_SWAY": "#fed7aa", "HOVER_CONFIRM": "#fde68a",
    "MARKER_SAVE": "#fde68a", "RESCUE_ROUTE_PLAN": "#f3e8ff",
    "RESCUE_VISIT": "#ddd6fe", "RETURN_HOME": "#bbf7d0",
    "VERTIPORT_ACQUIRE": "#a7f3d0", "VISION_SERVO_LAND": "#6ee7b7",
    "LANDED": "#4ade80", "EMERGENCY_RETURN": "#fca5a5", "ABORT": "#f87171",
}
_PHASE_SHORT = {
    "TAKEOFF": "TAKEOFF", "GRID_SEARCH": "SEARCH",
    "HOVER_CONFIRM": "HOVER", "RESCUE_VISIT": "RESCUE",
    "RETURN_HOME": "RETURN", "VISION_SERVO_LAND": "SERVO",
    "LANDED": "LANDED",
}

# Phase groups to merge for clean Gantt display
_TAKEOFF_GROUP  = {"TAKEOFF", "HOME_INIT"}
_APPROACH_GROUP = {"MARKER_APPROACH", "ANTI_SWAY", "HOVER_CONFIRM", "MARKER_SAVE"}
_RESCUE_GROUP   = {"RESCUE_ROUTE_PLAN", "RESCUE_VISIT"}
_RETURN_GROUP   = {"RETURN_HOME", "VERTIPORT_ACQUIRE"}


def _merge_phases(states: list[Sample]) -> list[Sample]:
    """Merge consecutive sub-phases into logical blocks for a clean Gantt."""
    groups = [
        (_TAKEOFF_GROUP,  "TAKEOFF"),
        (_APPROACH_GROUP, "HOVER_CONFIRM"),
        (_RESCUE_GROUP,   "RESCUE_VISIT"),
        (_RETURN_GROUP,   "RETURN_HOME"),
    ]
    result: list[Sample] = []
    i = 0
    while i < len(states):
        merged = False
        for gset, gname in groups:
            if states[i].value in gset:
                j = i
                while j < len(states) and states[j].value in gset:
                    j += 1
                result.append(Sample(states[i].t, gname))
                i = j
                merged = True
                break
        if not merged:
            result.append(states[i])
            i += 1
    return result


def _draw_phase_gantt(ax, states: list[Sample], total_s: float,
                      merged: bool = False) -> None:
    raw = _merge_phases(states) if merged else states
    for i, item in enumerate(raw):
        t0 = item.t
        t1 = raw[i + 1].t if i + 1 < len(raw) else total_s
        color = _PHASE_COLORS.get(item.value, "#cbd5e1")
        ax.barh(0, t1 - t0, left=t0, height=0.62,
                color=color, edgecolor="#94a3b8", linewidth=0.6, align="center")
        width = t1 - t0
        min_w = 2.2 if merged else 1.2
        if width > min_w:
            label = _PHASE_SHORT.get(item.value, item.value)
            ax.text((t0 + t1) / 2, 0, label,
                    ha="center", va="center", fontsize=7.0,
                    color="#1e293b", fontweight="bold")
    ax.set_yticks([])
    ax.set_ylim(-0.48, 0.48)



def _add_phase_bands(ax, states: list[Sample], total_s: float, alpha: float = 0.10) -> None:
    """Draw light colored vertical bands for each mission phase."""
    for i, item in enumerate(states):
        t0 = item.t
        t1 = states[i + 1].t if i + 1 < len(states) else total_s
        color = _PHASE_COLORS.get(item.value, "#cbd5e1")
        ax.axvspan(t0, t1, alpha=alpha, color=color, linewidth=0)


def pose_speed_samples(data: RunData) -> tuple[np.ndarray, np.ndarray]:
    if len(data.poses) <= 1:
        return np.asarray([]), np.asarray([])
    p = np.asarray(data.poses)
    dt = np.diff(p[:, 0])
    dx, dy = np.diff(p[:, 1]), np.diff(p[:, 2])
    speed = np.sqrt(dx**2 + dy**2) / np.where(dt > 0, dt, np.inf)
    t_mid = 0.5 * (p[:-1, 0] + p[1:, 0])
    return t_mid, np.clip(speed, 0, None)


def save_telemetry(data: RunData, out: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11.0, 7.2), dpi=220, sharex=True,
                             gridspec_kw={"height_ratios": [1.3, 1.1, 0.52]})
    duration = 0.0 if data.first_t is None or data.last_t is None else data.last_t - data.first_t

    # Key phase boundaries to annotate (label, time)
    _key = {"RESCUE_VISIT": "Rescue", "RETURN_HOME": "Return", "VISION_SERVO_LAND": "Land"}
    key_lines = [(item.t, _key[item.value]) for item in data.states if item.value in _key]

    def add_vlines(ax, top_label: bool = False):
        for t, label in key_lines:
            ax.axvline(t, color="#94a3b8", linewidth=0.9, linestyle="--", zorder=1)
            if top_label:
                ax.text(t + 0.3, ax.get_ylim()[1] * 0.96, label,
                        fontsize=7.2, color="#475569", va="top")

    # ── (a) Ground speed from pose diff ────────────────────────────────────
    t_mid, spd = pose_speed_samples(data)
    if len(spd):
        # Clip boundary noise from convolution
        kernel = np.ones(3) / 3
        spd_sm = np.convolve(spd, kernel, mode="same")
        spd_sm = np.clip(spd_sm, 0, None)

        axes[0].fill_between(t_mid, 0, spd_sm, color="#bfdbfe", alpha=0.55, zorder=2)
        axes[0].plot(t_mid, spd_sm, color="#1d4ed8", linewidth=1.7,
                     label="pose-derived speed", zorder=3)
        if data.planner_speeds:
            planner = np.asarray(data.planner_speeds)
            axes[0].step(planner[:, 0], planner[:, 1], where="post",
                         color="#b45309", linewidth=1.05, alpha=0.8,
                         label="planner target speed", zorder=4)
        axes[0].axhline(10.0, color="#64748b", linewidth=0.75, linestyle=":",
                        label="configured sprint target (10 m/s)", zorder=1)
        axes[0].set_ylim(0, 12.5)
        axes[0].set_yticks([0, 2, 4, 6, 8, 10])
        axes[0].legend(loc="lower right", fontsize=7.5, framealpha=0.9)
        add_vlines(axes[0], top_label=True)
    else:
        axes[0].text(0.5, 0.5, "No pose samples", transform=axes[0].transAxes,
                     ha="center", va="center", color="#344054")
    axes[0].set_ylabel("Speed (m/s)", fontsize=9)
    axes[0].set_title("(a) Pose-derived speed and planner target", fontsize=9.5)
    style_axes(axes[0])

    # ── (b) Altitude ────────────────────────────────────────────────────────
    alt_src = data.altitudes if data.altitudes else (
        [(p_[0], p_[3]) for p_ in data.poses] if data.poses else [])
    if alt_src:
        # Prepend synthetic takeoff point: gz_drone_sim starts at START_Z=0.3m.
        # Estimate launch time by back-projecting at 1.7 m/s climb rate.
        t0_alt, v0_alt = alt_src[0]
        t_launch = t0_alt - max(0.0, (v0_alt - 0.3) / 1.7)
        if t_launch < t0_alt - 0.05:
            alt_src = [(t_launch, 0.3)] + list(alt_src)
        arr = np.asarray(alt_src)
        axes[1].fill_between(arr[:, 0], 0, arr[:, 1], color="#bbf7d0", alpha=0.50, zorder=2)
        axes[1].plot(arr[:, 0], arr[:, 1], color="#15803d", linewidth=1.7, zorder=3)
        axes[1].axhline(2.0, color="#f97316", linestyle="--", linewidth=1.0,
                        label="2 m cruise altitude", zorder=1)
        axes[1].set_ylim(0, 2.55)
        axes[1].set_yticks([0, 0.5, 1.0, 1.5, 2.0])
        axes[1].legend(loc="upper right", fontsize=7.5, framealpha=0.9)
        add_vlines(axes[1])
    else:
        axes[1].text(0.5, 0.5, "No altitude samples", transform=axes[1].transAxes,
                     ha="center", va="center", color="#344054")
    axes[1].set_ylabel("Altitude (m)", fontsize=9)
    axes[1].set_title("(b) Altitude  —  2 m cruise → landing descent", fontsize=9.5)
    style_axes(axes[1])

    # ── (c) Phase Gantt ─────────────────────────────────────────────────────
    _draw_phase_gantt(axes[2], data.states, duration, merged=True)
    axes[2].set_xlabel("Time from bag start (s)", fontsize=9)
    axes[2].set_title("(c) Mission phase", fontsize=9.5)
    axes[2].set_xlim(0, duration * 1.01)
    style_axes(axes[2])

    landed_t = next((item.t for item in data.states if item.value == "LANDED"), duration)
    total_s = f"{landed_t:.1f} s"
    fig.suptitle(f"Headless 10 m/s 전체 미션 텔레메트리  ({total_s}, seed 310)",
                 fontsize=11, weight="bold", y=0.999)
    fig.tight_layout(h_pad=1.8)
    fig.savefig(out, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def write_summary(data: RunData, out: Path) -> None:
    duration = 0.0 if data.first_t is None or data.last_t is None else data.last_t - data.first_t
    max_step = 0.0
    if len(data.poses) > 1:
        pose = np.asarray(data.poses)
        max_step = float(np.max(np.hypot(np.diff(pose[:, 1]), np.diff(pose[:, 2]))))
    _, pose_speeds = pose_speed_samples(data)
    planner_speeds = np.asarray(data.planner_speeds)
    landed_t = next((item.t for item in data.states if item.value == "LANDED"), duration)
    speed_lines: list[str] = []
    if len(pose_speeds):
        speed_lines.append(
            f"- Pose-derived ground speed: mean {pose_speeds.mean():.3f} m/s, "
            f"p95 {np.percentile(pose_speeds, 95):.3f} m/s, max {pose_speeds.max():.3f} m/s"
        )
    if len(planner_speeds):
        speed_lines.append(
            f"- Planner target speed: mean {planner_speeds[:, 1].mean():.3f} m/s, "
            f"p95 {np.percentile(planner_speeds[:, 1], 95):.3f} m/s, "
            f"max {planner_speeds[:, 1].max():.3f} m/s"
        )
    out.write_text(
        "\n".join([
            "# Headless Mission Figure Data Summary",
            "",
            "> Fig. 8 is generated from the headless 10 m/s full-mission rosbag. Fig. 5/6 are regenerated from the mission-design plotters so they can show the complete marker layout, hover-confirm segments, rescue order, return, and landing sequence.",
            "",
            f"- Duration: {duration:.3f} s",
            f"- LANDED transition: {landed_t:.2f} s",
            f"- Pose samples: {len(data.poses)}",
            f"- Trajectory setpoints: {len(data.speeds)}",
            f"- Planner velocity profiles: {data.planner_count}",
            f"- Downward camera frames: {data.image_count}",
            f"- Grid detector messages: {data.grid_count}",
            f"- Marker messages: {data.marker_count}",
            f"- Max localization sample step: {max_step:.3f} m",
            *speed_lines,
            "- Mission transitions:",
            *[f"  - {item.t:.2f}s: {item.value}" for item in data.states],
            "- Controller transitions:",
            *[f"  - {item.t:.2f}s: {item.value}" for item in data.modes],
            "",
        ]),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()

    data = read_bag(args.bag, args.repo_root)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    save_system_block(args.out_dir / "fig_01_system_block.png")
    # Fig. 5/6 are mission-design figures. They must show the complete
    # serpentine search, marker hover-confirm segments, rescue sprint order,
    # return, and landing sequence, so they are generated by the dedicated
    # plotters in scripts/plot_phase1_search.py and scripts/plot_mission_path.py.
    save_telemetry(data, args.out_dir / "fig_08_gazebo_telemetry.png")
    write_summary(data, args.out_dir / "gazebo_figure_data_summary.md")

    print(f"Wrote figures to {args.out_dir}")
    print(f"  poses={len(data.poses)} images={data.image_count} states={len(data.states)}")


if __name__ == "__main__":
    main()
