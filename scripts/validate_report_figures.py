#!/usr/bin/env python3
"""Validate report figure references and the data used by the figures."""

from __future__ import annotations

import csv
import importlib.util
import re
import struct
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "first_round_technical_plan_latex_draft.tex"
WORLD = ROOT / "src" / "sprint_drone" / "worlds" / "sprint_grid_world.sdf"
SWEEP = ROOT / "reports" / "marker_layout_sweep.csv"
SPEED_SWEEP = ROOT / "reports" / "phase1_speed_sweep.csv"
TELEMETRY_SUMMARY = ROOT / "reports" / "gazebo_figure_data_summary.md"
BAG_DIR = ROOT / "bags" / "headless_20260517_204437" / "run"
BAG_METADATA = BAG_DIR / "metadata.yaml"
FIGURE_GENERATOR = ROOT / "reports" / "generate_report_figures.py"
OUT = ROOT / "reports" / "figure_validation_summary.md"

START_WORLD = (2.0, 19.0)
EXPECTED_WORLD = {
    1: (19.0, 19.0),
    2: (4.0, 16.0),
    3: (10.0, 4.0),
    4: (25.0, 4.0),
}
EXPECTED_GRID = {
    mid: (x - START_WORLD[0], y - START_WORLD[1])
    for mid, (x, y) in EXPECTED_WORLD.items()
}


def png_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path} is not a PNG file")
    return struct.unpack(">II", data[16:24])


def parse_report_figures() -> list[str]:
    text = REPORT.read_text(encoding="utf-8")
    return re.findall(r"\\includegraphics(?:\[[^\]]*\])?\{(fig_[^}]+\.png)\}", text)


def parse_world_markers() -> dict[int, tuple[float, float]]:
    text = WORLD.read_text(encoding="utf-8")
    pattern = re.compile(
        r'<model name="aruco_marker_(\d+)">.*?<pose>([-\d.]+) ([-\d.]+) ',
        re.DOTALL,
    )
    return {
        int(mid): (float(x), float(y))
        for mid, x, y in pattern.findall(text)
    }


def load_sweep() -> list[dict[str, str]]:
    with SWEEP.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_speed_sweep() -> list[dict[str, str]]:
    with SPEED_SWEEP.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def row_for_seed(rows: list[dict[str, str]], seed: int) -> dict[str, str]:
    matches = [row for row in rows if int(row["seed"]) == seed]
    if len(matches) != 1:
        raise ValueError(f"expected one sweep row for seed {seed}, found {len(matches)}")
    return matches[0]


def metadata_topic_counts() -> tuple[float, dict[str, int]]:
    info = yaml.safe_load(BAG_METADATA.read_text(encoding="utf-8"))["rosbag2_bagfile_information"]
    duration_s = info["duration"]["nanoseconds"] / 1e9
    counts: dict[str, int] = {}
    for item in info["topics_with_message_count"]:
        counts[item["topic_metadata"]["name"]] = item["message_count"]
    return duration_s, counts


def load_figure_generator_data():
    spec = importlib.util.spec_from_file_location("report_figure_generator", FIGURE_GENERATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {FIGURE_GENERATOR}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["report_figure_generator"] = module
    spec.loader.exec_module(module)
    return module, module.read_bag(BAG_DIR, ROOT)


def check() -> tuple[list[str], list[str]]:
    ok: list[str] = []
    warn: list[str] = []

    fig_refs = parse_report_figures()
    for name in fig_refs:
        path = ROOT / "reports" / name
        if not path.exists():
            warn.append(f"missing figure: {name}")
            continue
        width, height = png_size(path)
        if width < 900 or height < 450:
            warn.append(f"low-resolution figure: {name} ({width}x{height})")
        else:
            ok.append(f"{name}: {width}x{height}")

    world_markers = parse_world_markers()
    if world_markers == EXPECTED_WORLD:
        ok.append("Gazebo world marker positions match seed 310")
    else:
        warn.append(f"Gazebo world marker mismatch: {world_markers}")

    rows = load_sweep()
    if len(rows) == 600:
        ok.append("sweep CSV contains 600 random-layout cases")
    else:
        warn.append(f"sweep CSV row count is {len(rows)}, expected 600")

    seed310 = row_for_seed(rows, 310)
    for mid in (1, 2, 3, 4):
        expected = EXPECTED_GRID[mid]
        got = seed310[f"marker_{mid}_grid"]
        expected_text = f"({expected[0]:.0f},{expected[1]:.0f})"
        if got != expected_text:
            warn.append(f"seed 310 marker {mid} grid mismatch: {got} != {expected_text}")
    if not any("seed 310 marker" in item for item in warn):
        ok.append("sweep seed 310 marker grid coordinates match Gazebo world")

    classes = Counter(row["layout_class"] for row in rows)
    ok.append(
        "sweep class counts: "
        + ", ".join(f"{name}={classes[name]}" for name in ("clustered", "mixed", "spread"))
    )

    speed_rows = load_speed_sweep()
    expected_speed_rows = 600 * 3
    if len(speed_rows) == expected_speed_rows:
        ok.append(f"Phase 1 speed sweep CSV contains {expected_speed_rows} cases")
    else:
        warn.append(f"Phase 1 speed sweep row count is {len(speed_rows)}, expected {expected_speed_rows}")
    speeds = sorted({row["phase1_speed_mps"] for row in speed_rows}, key=float)
    if speeds == ["5.0", "7.0", "10.0"]:
        ok.append("Phase 1 speed sweep covers 5/7/10 m/s")
    else:
        warn.append(f"Phase 1 speed sweep speeds mismatch: {speeds}")

    summary = TELEMETRY_SUMMARY.read_text(encoding="utf-8")
    for key in (
        "Duration:",
        "LANDED transition:",
        "Pose samples:",
        "Planner velocity profiles:",
        "Downward camera frames:",
    ):
        if key in summary:
            ok.append(f"Telemetry summary contains {key}")
        else:
            warn.append(f"Telemetry summary missing {key}")

    metadata_duration_s, topic_counts = metadata_topic_counts()
    figure_module, bag_data = load_figure_generator_data()
    expected_counts = {
        "Fig. 8 pose source": (
            len(bag_data.poses),
            topic_counts.get("/localization/pose_grid", 0),
        ),
        "Fig. 8 planner target source": (
            bag_data.planner_count,
            topic_counts.get("/planner/velocity_profile", 0),
        ),
        "Fig. 8 setpoint speed source": (
            len(bag_data.speeds),
            topic_counts.get("/fmu/in/trajectory_setpoint", 0),
        ),
        "Fig. 8 altitude source": (
            len(bag_data.altitudes),
            topic_counts.get("/fmu/out/vehicle_local_position", 0),
        ),
        "Fig. 8 camera source": (
            bag_data.image_count,
            topic_counts.get("/drone/camera/down/image_raw", 0),
        ),
        "Fig. 8 grid source": (
            bag_data.grid_count,
            topic_counts.get("/grid/intersections", 0),
        ),
        "Fig. 8 marker source": (
            bag_data.marker_count,
            topic_counts.get("/markers/confirmed", 0),
        ),
    }
    for name, (actual, expected) in expected_counts.items():
        if actual == expected:
            ok.append(f"{name}: {actual} samples match rosbag metadata")
        else:
            warn.append(f"{name}: generator={actual}, metadata={expected}")

    duration_from_generator = 0.0 if bag_data.first_t is None or bag_data.last_t is None else bag_data.last_t - bag_data.first_t
    if abs(duration_from_generator - metadata_duration_s) < 0.01:
        ok.append(f"Fig. 8 duration matches metadata: {duration_from_generator:.3f}s")
    else:
        warn.append(f"duration mismatch: generator={duration_from_generator:.3f}s metadata={metadata_duration_s:.3f}s")

    _, pose_speed = figure_module.pose_speed_samples(bag_data)
    speed = np.asarray(bag_data.speeds)
    planner_speed = np.asarray(bag_data.planner_speeds)
    altitude = np.asarray(bag_data.altitudes)
    pose = np.asarray(bag_data.poses)
    if len(pose_speed):
        ok.append(
            f"Fig. 8 pose-derived speed data: n={len(pose_speed)}, "
            f"mean={pose_speed.mean():.3f}m/s, p95={np.percentile(pose_speed, 95):.3f}m/s, "
            f"max={pose_speed.max():.3f}m/s"
        )
        if np.percentile(pose_speed, 95) < 9.5:
            warn.append("Fig. 8 pose-derived speed p95 is below the 10 m/s target envelope")
    else:
        warn.append("Fig. 8 pose-derived speed data is empty")
    if len(planner_speed):
        ok.append(
            f"Fig. 8 planner target speed data: n={len(planner_speed)}, "
            f"min={planner_speed[:,1].min():.3f}m/s, p95={np.percentile(planner_speed[:,1], 95):.3f}m/s, "
            f"max={planner_speed[:,1].max():.3f}m/s"
        )
        if abs(float(planner_speed[:, 1].max()) - 10.0) > 0.01:
            warn.append("Fig. 8 planner target speed does not reach 10 m/s")
    else:
        warn.append("Fig. 8 planner target speed data is empty")
    if len(speed):
        ok.append(
            f"Fig. 8 speed data: n={len(speed)}, min={speed[:,1].min():.3f}m/s, "
            f"max={speed[:,1].max():.3f}m/s"
        )
    if len(altitude):
        ok.append(
            f"Fig. 8 altitude data: n={len(altitude)}, min={altitude[:,1].min():.3f}m, "
            f"max={altitude[:,1].max():.3f}m, final={altitude[-1,1]:.3f}m"
        )
    if len(pose) > 1:
        steps = np.hypot(np.diff(pose[:, 1]), np.diff(pose[:, 2]))
        ok.append(f"Fig. 8 run summary max pose sample step: {steps.max():.3f}m")
    ok.append(
        "Fig. 8 transitions: "
        f"mission={[(round(i.t, 2), i.value) for i in bag_data.states]}, "
        f"controller={[(round(i.t, 2), i.value) for i in bag_data.modes]}"
    )

    return ok, warn


def main() -> None:
    ok, warn = check()
    lines = [
        "# Figure Validation Summary",
        "",
        "## Passed checks",
        *[f"- {item}" for item in ok],
        "",
        "## Warnings",
    ]
    if warn:
        lines.extend(f"- {item}" for item in warn)
    else:
        lines.append("- none")
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Validation written to {OUT}")
    if warn:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
