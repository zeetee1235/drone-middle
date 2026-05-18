#!/usr/bin/env python3
"""Generate report-only figures and marker-layout sweep artifacts.

The existing Gazebo run remains the source for Fig. 8. This script fills
the missing design figures and runs a deterministic mission-level sweep over
random ArUco marker placements on the same 54 Gazebo grid intersections.
"""

from __future__ import annotations

import csv
import math
import random
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path
from statistics import mean, median

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D


ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "reports"

KO_FONT_PATH = Path("/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf")
if KO_FONT_PATH.exists():
    fm.fontManager.addfont(str(KO_FONT_PATH))
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=str(KO_FONT_PATH)).get_name()

matplotlib.rcParams["axes.unicode_minus"] = False
matplotlib.rcParams.update({
    "figure.dpi": 220,
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


GRID_ORIGIN_X = 4.0
GRID_ORIGIN_Y = 4.0
GRID_SPACING = 3.0
GRID_COLS = 9
GRID_ROWS = 6
START_WORLD = (2.0, 19.0)
HOME_GRID = (0.0, 0.0)
CURRENT_SEED = 310
SWEEP_CASES = 600

SEARCH_SPEED_MPS = 10.0
PHASE1_SPEED_SWEEP_MPS = (5.0, 7.0, 10.0)
SPRINT_SPEED_MPS = 10.0
RETURN_SPEED_MPS = 10.0
# Calibrated from headless_20260517_204437 (LANDED=52.6 s, seed 310)
# Pass-through detection → overhead ≈ 0; hover = 4 markers × 3.0 s each
TAKEOFF_AND_INIT_S = 1.5         # observed ~1.4 s
SEARCH_HOVER_S = 12.0            # 4 × hover_confirm_duration (3.0 s) — unchanged
SEARCH_ACCEL_OVERHEAD_S = 0.5    # candidate detection, minimal extra stop in the cost model
RESCUE_HOVER_S = 12.0            # 4 × hover_confirm_duration (3.0 s) — unchanged
RESCUE_ACCEL_OVERHEAD_S = 0.5    # was 10.0 s; sprint mode, near-zero overhead
LANDING_S = 4.0                  # was 5.0 s; observed visual-servo landing ~3.8 s
CAMERA_FPS = 60.0
MARKER_VISIBLE_DISTANCE_M = 3.0
MIN_DETECTION_FRAMES = 20.0

PALETTE = {
    "navy": "#22324a",
    "blue": "#2563eb",
    "cyan": "#0891b2",
    "green": "#16a34a",
    "orange": "#f97316",
    "red": "#dc2626",
    "violet": "#7c3aed",
    "slate": "#475569",
    "grid": "#cbd5e1",
    "light": "#f8fafc",
    "line": "#334155",
}


@dataclass(frozen=True)
class Marker:
    marker_id: int
    world_x: float
    world_y: float

    @property
    def grid(self) -> tuple[float, float]:
        return self.world_x - START_WORLD[0], self.world_y - START_WORLD[1]


@dataclass(frozen=True)
class SweepResult:
    seed: int
    layout_class: str
    min_pair_distance_m: float
    mean_pair_distance_m: float
    span_x_m: float
    span_y_m: float
    all_found_index: int
    search_distance_m: float
    search_time_s: float
    phase0_time_s: float
    phase1_time_s: float
    phase2_distance_m: float
    phase2_travel_time_s: float
    phase2_time_s: float
    return_distance_m: float
    return_time_s: float
    phase3_time_s: float
    rescue_return_distance_m: float
    rescue_return_time_s: float
    total_time_s: float
    markers: tuple[Marker, ...]


def style_axes(ax) -> None:
    ax.grid(True, color="#d0d5dd", linewidth=0.55, alpha=0.7)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_linewidth(0.8)


def ensure_dir() -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)


def savefig(path: Path, *, tight: bool = True) -> None:
    if tight:
        plt.savefig(path, bbox_inches="tight", facecolor="white")
    else:
        plt.savefig(path, facecolor="white")
    plt.close()


def draw_box(ax, xy, wh, title, body="", color="#e0f2fe", ec="#1e293b", lw=1.2, fs=10):
    x, y = xy
    w, h = wh
    box = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.025,rounding_size=0.08",
        linewidth=lw,
        edgecolor=ec,
        facecolor=color,
    )
    ax.add_patch(box)
    ax.text(x + w / 2, y + h * 0.62, title, ha="center", va="center", fontsize=fs, weight="bold", color="#111827")
    if body:
        ax.text(x + w / 2, y + h * 0.32, body, ha="center", va="center", fontsize=fs - 1, color="#334155")
    return box


def draw_arrow(ax, start, end, color="#334155", lw=1.8, text=None, text_offset=(0, 0)):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, shrinkA=4, shrinkB=4),
    )
    if text:
        mx = (start[0] + end[0]) / 2 + text_offset[0]
        my = (start[1] + end[1]) / 2 + text_offset[1]
        ax.text(mx, my, text, ha="center", va="center", fontsize=8.5, color=color)


def draw_poly_arrow(ax, points, color="#334155", lw=1.8):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ax.plot(xs[:-1], ys[:-1], color=color, lw=lw)
    ax.annotate(
        "",
        xy=points[-1],
        xytext=points[-2],
        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, shrinkA=0, shrinkB=4),
    )


def all_intersections() -> list[tuple[float, float]]:
    return [
        (GRID_ORIGIN_X + col * GRID_SPACING, GRID_ORIGIN_Y + row * GRID_SPACING)
        for col in range(GRID_COLS)
        for row in range(GRID_ROWS)
    ]


def pick_markers(seed: int) -> tuple[Marker, ...]:
    rng = random.Random(seed)
    chosen = rng.sample(all_intersections(), 4)
    return tuple(Marker(mid, x, y) for mid, (x, y) in zip((1, 2, 3, 4), chosen))


def build_serpentine_grid() -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for row in range(GRID_ROWS):
        y = 0.0 - row * GRID_SPACING
        cols = range(GRID_COLS) if row % 2 == 0 else range(GRID_COLS - 1, -1, -1)
        for col in cols:
            pts.append((2.0 + col * GRID_SPACING, y))
    return pts


SERPENTINE_GRID = build_serpentine_grid()


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_distance(points: list[tuple[float, float]]) -> float:
    return sum(distance(a, b) for a, b in zip(points[:-1], points[1:]))


def classify_layout(min_pair: float) -> str:
    if min_pair <= 4.3:
        return "clustered"
    if min_pair >= 9.0:
        return "spread"
    return "mixed"


def evaluate_layout(seed: int, search_speed_mps: float = SEARCH_SPEED_MPS) -> SweepResult:
    markers = pick_markers(seed)
    grid_positions = {m.marker_id: m.grid for m in markers}
    pair_distances = [distance(a.grid, b.grid) for a, b in combinations(markers, 2)]
    xs = [m.grid[0] for m in markers]
    ys = [m.grid[1] for m in markers]
    min_pair = min(pair_distances)
    mean_pair = mean(pair_distances)
    layout_class = classify_layout(min_pair)

    found_indices = [SERPENTINE_GRID.index(grid_positions[mid]) for mid in grid_positions]
    all_found_index = max(found_indices)
    search_points = [HOME_GRID] + SERPENTINE_GRID[: all_found_index + 1]
    search_distance = path_distance(search_points)
    search_time = search_distance / search_speed_mps
    phase0_time = TAKEOFF_AND_INIT_S
    phase1_time = search_time + SEARCH_ACCEL_OVERHEAD_S + SEARCH_HOVER_S

    current = SERPENTINE_GRID[all_found_index]
    phase2_points = [current]
    for marker_id in (4, 3, 2, 1):
        phase2_points.append(grid_positions[marker_id])
    phase2_distance = path_distance(phase2_points)
    phase2_travel_time = phase2_distance / SPRINT_SPEED_MPS
    phase2_time = phase2_travel_time + RESCUE_ACCEL_OVERHEAD_S + RESCUE_HOVER_S
    return_distance = distance(phase2_points[-1], HOME_GRID)
    return_time = return_distance / RETURN_SPEED_MPS
    phase3_time = return_time + LANDING_S

    rescue_return_distance = phase2_distance + return_distance
    rescue_return_time = phase2_travel_time + return_time
    total_time = phase0_time + phase1_time + phase2_time + phase3_time

    return SweepResult(
        seed=seed,
        layout_class=layout_class,
        min_pair_distance_m=min_pair,
        mean_pair_distance_m=mean_pair,
        span_x_m=max(xs) - min(xs),
        span_y_m=max(ys) - min(ys),
        all_found_index=all_found_index,
        search_distance_m=search_distance,
        search_time_s=search_time,
        phase0_time_s=phase0_time,
        phase1_time_s=phase1_time,
        phase2_distance_m=phase2_distance,
        phase2_travel_time_s=phase2_travel_time,
        phase2_time_s=phase2_time,
        return_distance_m=return_distance,
        return_time_s=return_time,
        phase3_time_s=phase3_time,
        rescue_return_distance_m=rescue_return_distance,
        rescue_return_time_s=rescue_return_time,
        total_time_s=total_time,
        markers=markers,
    )


def run_sweep() -> list[SweepResult]:
    results = [evaluate_layout(seed) for seed in range(SWEEP_CASES)]
    if CURRENT_SEED >= SWEEP_CASES:
        results.append(evaluate_layout(CURRENT_SEED))
    return results


def write_sweep_csv(results: list[SweepResult]) -> None:
    path = REPORT_DIR / "marker_layout_sweep.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "seed",
                "layout_class",
                "min_pair_distance_m",
                "mean_pair_distance_m",
                "span_x_m",
                "span_y_m",
                "all_found_index",
                "search_distance_m",
                "search_time_s",
                "phase0_time_s",
                "phase1_time_s",
                "phase2_distance_m",
                "phase2_travel_time_s",
                "phase2_time_s",
                "return_distance_m",
                "return_time_s",
                "phase3_time_s",
                "rescue_return_distance_m",
                "rescue_return_time_s",
                "total_time_s",
                "marker_1_grid",
                "marker_2_grid",
                "marker_3_grid",
                "marker_4_grid",
            ]
        )
        for r in results:
            grid_by_id = {m.marker_id: m.grid for m in r.markers}
            writer.writerow(
                [
                    r.seed,
                    r.layout_class,
                    f"{r.min_pair_distance_m:.3f}",
                    f"{r.mean_pair_distance_m:.3f}",
                    f"{r.span_x_m:.3f}",
                    f"{r.span_y_m:.3f}",
                    r.all_found_index,
                    f"{r.search_distance_m:.3f}",
                    f"{r.search_time_s:.3f}",
                    f"{r.phase0_time_s:.3f}",
                    f"{r.phase1_time_s:.3f}",
                    f"{r.phase2_distance_m:.3f}",
                    f"{r.phase2_travel_time_s:.3f}",
                    f"{r.phase2_time_s:.3f}",
                    f"{r.return_distance_m:.3f}",
                    f"{r.return_time_s:.3f}",
                    f"{r.phase3_time_s:.3f}",
                    f"{r.rescue_return_distance_m:.3f}",
                    f"{r.rescue_return_time_s:.3f}",
                    f"{r.total_time_s:.3f}",
                    f"({grid_by_id[1][0]:.0f},{grid_by_id[1][1]:.0f})",
                    f"({grid_by_id[2][0]:.0f},{grid_by_id[2][1]:.0f})",
                    f"({grid_by_id[3][0]:.0f},{grid_by_id[3][1]:.0f})",
                    f"({grid_by_id[4][0]:.0f},{grid_by_id[4][1]:.0f})",
                ]
            )


def grouped(results: list[SweepResult]) -> dict[str, list[SweepResult]]:
    out = {"clustered": [], "mixed": [], "spread": []}
    for result in results:
        out[result.layout_class].append(result)
    return out


def write_sweep_summary(results: list[SweepResult]) -> None:
    current = evaluate_layout(CURRENT_SEED)
    groups = grouped(results)
    total_times = [r.total_time_s for r in results]
    lines = [
        "# ArUco 랜덤 배치 sweep 요약",
        "",
        f"- sweep 범위: seed 0~{SWEEP_CASES - 1} (현재 Gazebo seed {CURRENT_SEED} 포함)",
        f"- 후보 위치: Gazebo 경기장 3m 격자 교차점 {GRID_COLS * GRID_ROWS}개",
        f"- 모델 속도: 탐색 {SEARCH_SPEED_MPS:.1f}m/s, 구조/귀환 스프린트 {SPRINT_SPEED_MPS:.1f}/{RETURN_SPEED_MPS:.1f}m/s",
        f"- 고정/오버헤드: Phase0 {TAKEOFF_AND_INIT_S:.0f}초, 탐색 호버 {SEARCH_HOVER_S:.0f}초, 탐색 감속/재가속 {SEARCH_ACCEL_OVERHEAD_S:.0f}초, 구조 호버 {RESCUE_HOVER_S:.0f}초, 구조 감속/재가속 {RESCUE_ACCEL_OVERHEAD_S:.0f}초, 착륙 {LANDING_S:.0f}초",
        f"- 전체 모델 시간: 최소 {min(total_times):.1f}s, 중앙값 {median(total_times):.1f}s, 최대 {max(total_times):.1f}s",
        "",
        "| 배치 유형 | 개수 | 중앙 시간(s) | 중앙 Phase1(s) | 중앙 Phase2(s) | 중앙 Phase3(s) | 중앙 탐색거리(m) | 중앙 구조+귀환거리(m) |",
        "|-----------|------|--------------|-----------------|-----------------|-----------------|------------------|------------------------|",
    ]
    for name in ("clustered", "mixed", "spread"):
        vals = groups[name]
        lines.append(
            f"| {name} | {len(vals)} | {median([v.total_time_s for v in vals]):.1f} | "
            f"{median([v.phase1_time_s for v in vals]):.1f} | "
            f"{median([v.phase2_time_s for v in vals]):.1f} | "
            f"{median([v.phase3_time_s for v in vals]):.1f} | "
            f"{median([v.search_distance_m for v in vals]):.1f} | "
            f"{median([v.rescue_return_distance_m for v in vals]):.1f} |"
        )
    lines.extend(
        [
            "",
            "## 현재 Gazebo seed 310",
            "",
            f"- 배치 유형: {current.layout_class}",
            f"- 최소 마커 간 거리: {current.min_pair_distance_m:.1f}m",
            f"- 4개 마커 발견까지 교차점 index: {current.all_found_index + 1}/54",
            f"- 모델 탐색 거리: {current.search_distance_m:.1f}m",
            f"- 모델 구조+귀환 거리: {current.rescue_return_distance_m:.1f}m",
            f"- Phase 0/1/2/3 시간: {current.phase0_time_s:.1f}s / {current.phase1_time_s:.1f}s / {current.phase2_time_s:.1f}s / {current.phase3_time_s:.1f}s",
            f"- 모델 총 시간: {current.total_time_s:.1f}s",
        ]
    )
    (REPORT_DIR / "marker_layout_sweep_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def phase1_speed_sweep() -> dict[float, list[SweepResult]]:
    return {
        speed: [evaluate_layout(seed, search_speed_mps=speed) for seed in range(SWEEP_CASES)]
        for speed in PHASE1_SPEED_SWEEP_MPS
    }


def marker_exposure_frames(speed_mps: float) -> float:
    return CAMERA_FPS * MARKER_VISIBLE_DISTANCE_M / speed_mps


def write_phase1_speed_sweep_csv(speed_results: dict[float, list[SweepResult]]) -> None:
    path = REPORT_DIR / "phase1_speed_sweep.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "phase1_speed_mps",
                "seed",
                "layout_class",
                "all_found_index",
                "search_distance_m",
                "phase1_time_s",
                "phase2_time_s",
                "phase3_time_s",
                "total_time_s",
                "marker_visible_time_s",
                "marker_visible_frames_60fps",
                "frame_margin_vs_20",
            ]
        )
        for speed, results in speed_results.items():
            visible_time = MARKER_VISIBLE_DISTANCE_M / speed
            visible_frames = marker_exposure_frames(speed)
            frame_margin = visible_frames / MIN_DETECTION_FRAMES
            for result in results:
                writer.writerow(
                    [
                        f"{speed:.1f}",
                        result.seed,
                        result.layout_class,
                        result.all_found_index,
                        f"{result.search_distance_m:.3f}",
                        f"{result.phase1_time_s:.3f}",
                        f"{result.phase2_time_s:.3f}",
                        f"{result.phase3_time_s:.3f}",
                        f"{result.total_time_s:.3f}",
                        f"{visible_time:.3f}",
                        f"{visible_frames:.1f}",
                        f"{frame_margin:.2f}",
                    ]
                )


def write_phase1_speed_sweep_summary(speed_results: dict[float, list[SweepResult]]) -> None:
    lines = [
        "# Phase 1 속도 sweep 요약",
        "",
        f"- sweep 범위: seed 0~{SWEEP_CASES - 1}, Phase 1 속도 {', '.join(f'{v:.0f}m/s' for v in PHASE1_SPEED_SWEEP_MPS)}",
        f"- 공통 가정: 구조/귀환 {SPRINT_SPEED_MPS:.1f}/{RETURN_SPEED_MPS:.1f}m/s, 탐색 호버 {SEARCH_HOVER_S:.0f}초, 구조 호버 {RESCUE_HOVER_S:.0f}초, 탐색 감속/재가속 {SEARCH_ACCEL_OVERHEAD_S:.0f}초",
        f"- ArUco 노출 모델: 60fps, 가시구간 {MARKER_VISIBLE_DISTANCE_M:.1f}m, 기준 최소 프레임 {MIN_DETECTION_FRAMES:.0f} frames",
        "",
        "| Phase 1 속도 | 최소(s) | 평균(s) | 중앙값(s) | 최대(s) | 현재 seed 310(s) | 70초 이하 | 80초 이하 | 노출 시간(s) | 60fps 프레임 |",
        "|--------------|---------|---------|-----------|---------|------------------|----------|----------|-------------|-------------|",
    ]
    for speed in PHASE1_SPEED_SWEEP_MPS:
        results = speed_results[speed]
        totals = [r.total_time_s for r in results]
        current = next(r for r in results if r.seed == CURRENT_SEED)
        under70 = sum(t <= 70.0 for t in totals)
        under80 = sum(t <= 80.0 for t in totals)
        visible_time = MARKER_VISIBLE_DISTANCE_M / speed
        visible_frames = marker_exposure_frames(speed)
        lines.append(
            f"| {speed:.0f}m/s | {min(totals):.1f} | {mean(totals):.1f} | {median(totals):.1f} | {max(totals):.1f} | "
            f"{current.total_time_s:.1f} | {under70}/{len(results)} | {under80}/{len(results)} | "
            f"{visible_time:.2f} | {visible_frames:.1f} |"
        )
    lines.extend(
        [
            "",
            "## 해석",
            "",
            "- 이 결과는 마커 인식 성공률을 직접 측정한 값이 아니라 mission-cost 모델과 카메라 노출 프레임 수를 결합한 속도-시간 trade-off다.",
            "- 실제 한계는 속도별 ArUco 영상 검출 성공률을 별도 실험으로 측정해 위 프레임 수 모델을 보정해야 한다.",
        ]
    )
    (REPORT_DIR / "phase1_speed_sweep_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_fig_13_phase1_speed_tradeoff(speed_results: dict[float, list[SweepResult]]) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8))
    speeds = list(PHASE1_SPEED_SWEEP_MPS)
    labels = [f"{speed:.0f}" for speed in speeds]
    totals_by_speed = [[r.total_time_s for r in speed_results[speed]] for speed in speeds]
    current_totals = [next(r for r in speed_results[speed] if r.seed == CURRENT_SEED).total_time_s for speed in speeds]

    ax = axes[0]
    bp = ax.boxplot(totals_by_speed, tick_labels=labels, patch_artist=True, showfliers=False)
    for patch, color in zip(bp["boxes"], [PALETTE["orange"], PALETTE["blue"], PALETTE["green"]]):
        patch.set_facecolor(color)
        patch.set_alpha(0.58)
        patch.set_edgecolor(PALETTE["line"])
    for med in bp["medians"]:
        med.set_color("#111827")
        med.set_linewidth(1.8)
    ax.plot(range(1, len(speeds) + 1), current_totals, marker="*", color=PALETTE["red"], lw=1.6, label="current seed 310")
    ax.set_xlabel("Phase 1 search speed (m/s)")
    ax.set_ylabel("modeled mission time (s)")
    ax.set_title("(a) Mission time distribution")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(fontsize=8, frameon=False)

    ax = axes[1]
    thresholds = [70.0, 80.0]
    x = list(range(len(speeds)))
    width = 0.34
    for i, threshold in enumerate(thresholds):
        counts = [sum(r.total_time_s <= threshold for r in speed_results[speed]) for speed in speeds]
        offset = (i - 0.5) * width
        ax.bar([v + offset for v in x], counts, width=width, label=f"<= {threshold:.0f}s", alpha=0.75)
        for xpos, count in zip([v + offset for v in x], counts):
            ax.text(xpos, count + 3, str(count), ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x, labels)
    ax.set_xlabel("Phase 1 search speed (m/s)")
    ax.set_ylabel("cases out of 600")
    ax.set_title("(b) Fast-run feasibility count")
    ax.set_ylim(0, SWEEP_CASES * 1.08)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(fontsize=8, frameon=False)

    ax = axes[2]
    visible_times = [MARKER_VISIBLE_DISTANCE_M / speed for speed in speeds]
    visible_frames = [marker_exposure_frames(speed) for speed in speeds]
    ax.plot(speeds, visible_frames, marker="o", color=PALETTE["violet"], lw=2.0, label="60fps visible frames")
    ax.axhline(MIN_DETECTION_FRAMES, color=PALETTE["red"], ls="--", lw=1.2, label=f"{MIN_DETECTION_FRAMES:.0f}-frame reference")
    for speed, frames, visible_time in zip(speeds, visible_frames, visible_times):
        ax.text(speed, frames + 1.2, f"{frames:.1f}f\n{visible_time:.2f}s", ha="center", va="bottom", fontsize=8)
    ax.set_xlabel("Phase 1 search speed (m/s)")
    ax.set_ylabel("marker exposure frames")
    ax.set_title("(c) ArUco exposure proxy")
    ax.set_xticks(speeds)
    ax.set_ylim(0, max(visible_frames) * 1.35)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, frameon=False)

    fig.suptitle("Phase 1 속도-시간 Trade-off", fontsize=13, weight="bold")
    savefig(REPORT_DIR / "fig_13_phase1_speed_tradeoff.png")


def save_fig_02_mission_flow() -> None:
    fig, ax = plt.subplots(figsize=(12.5, 3.6))
    ax.set_xlim(0, 12.5)
    ax.set_ylim(0, 3.6)
    ax.axis("off")

    phases = [
        ("Phase 0", "Takeoff / Home\n2 m initialization", "#dbeafe"),
        ("Phase 1", "Grid Search\nintersection sprint", "#dcfce7"),
        ("Trigger", "4 markers found\nstop search immediately", "#ffedd5"),
        ("Phase 2", "Direct Rescue\nID 4→3→2→1", "#f3e8ff"),
        ("Phase 3", "Return / Land\nvisual servo landing", "#e0f2fe"),
    ]
    x = 0.35
    widths = [2.0, 2.25, 2.05, 2.25, 2.1]
    centers = []
    for (title, body, color), width in zip(phases, widths):
        draw_box(ax, (x, 1.25), (width, 1.25), title, body, color=color, fs=10)
        centers.append((x + width / 2, 1.875))
        x += width + 0.35

    for a, b in zip(centers[:-1], centers[1:]):
        draw_arrow(ax, (a[0] + 0.85, a[1]), (b[0] - 0.85, b[1]))

    ax.text(3.55, 0.58, "Optical Flow + grid-node snap, no line PID", ha="center", fontsize=9.0, color=PALETTE["green"])
    ax.text(8.25, 0.58, "Ignore grid lines, sprint through stored coordinates", ha="center", fontsize=9.0, color=PALETTE["violet"])
    ax.plot([2.8, 5.55], [0.85, 0.85], color=PALETTE["green"], lw=2.0)
    ax.plot([7.05, 9.85], [0.85, 0.85], color=PALETTE["violet"], lw=2.0)
    ax.set_title("탐색/구조 분리 임무 흐름", fontsize=13, weight="bold", color="#111827")
    savefig(REPORT_DIR / "fig_02_mission_flow.png")


def save_fig_03_optical_flow_snap() -> None:
    fig, ax = plt.subplots(figsize=(12.5, 5.2))
    ax.set_xlim(0, 12.5)
    ax.set_ylim(0, 5.2)
    ax.axis("off")

    draw_box(ax, (0.35, 3.35), (2.15, 0.95), "Downward Camera", "frame t, t+1", "#e0f2fe")
    draw_box(ax, (0.35, 1.95), (2.15, 0.95), "ToF Range", "pixel → meter", "#dcfce7")
    draw_box(ax, (0.35, 0.55), (2.15, 0.95), "IMU Attitude", "yaw / tilt correction", "#fef3c7")
    draw_box(ax, (3.15, 2.35), (2.35, 1.15), "Optical Flow\nIntegrator", "accumulate Δx, Δy", "#dbeafe")
    draw_box(ax, (6.1, 2.35), (2.35, 1.15), "Grid Node Snap", "reset drift at\nintersections", "#fee2e2")
    draw_box(ax, (9.05, 2.35), (2.55, 1.15), "Grid-frame Pose", "next setpoint\nmarker memory", "#f3e8ff")

    draw_arrow(ax, (2.5, 3.82), (3.15, 3.06))
    draw_arrow(ax, (2.5, 2.42), (3.15, 2.92))
    draw_arrow(ax, (2.5, 1.02), (3.15, 2.55))
    draw_arrow(ax, (5.5, 2.92), (6.1, 2.92))
    draw_arrow(ax, (8.45, 2.92), (9.05, 2.92))

    ax.plot([6.35, 6.35, 4.25, 4.25], [2.35, 1.35, 1.35, 2.35], color=PALETTE["red"], lw=1.7, linestyle="--")
    ax.text(5.3, 1.12, "restart integration after snap", ha="center", fontsize=8.8, color=PALETTE["red"])

    grid_ax = fig.add_axes([0.58, 0.08, 0.33, 0.26])
    for x in range(6):
        grid_ax.plot([x, x], [0, 3], color=PALETTE["grid"], lw=0.9)
    for y in range(4):
        grid_ax.plot([0, 5], [y, y], color=PALETTE["grid"], lw=0.9)
    grid_ax.plot([0.15, 1.0, 1.9, 3.0, 4.0], [2.0, 2.05, 1.92, 2.03, 2.0], color=PALETTE["blue"], lw=2.3, label="OF estimate")
    grid_ax.scatter([1, 2, 3, 4], [2, 2, 2, 2], s=58, color=PALETTE["orange"], zorder=3, label="snap point")
    grid_ax.set_xlim(-0.1, 5.1)
    grid_ax.set_ylim(-0.1, 3.1)
    grid_ax.set_aspect("equal")
    grid_ax.set_xticks([])
    grid_ax.set_yticks([])
    grid_ax.set_title("drift reset at each grid node", fontsize=8.5)
    grid_ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=7, frameon=False)

    ax.set_title("Optical Flow 기반 격자 좌표 보정", fontsize=13, weight="bold", color="#111827")
    savefig(REPORT_DIR / "fig_03_optical_flow_snap.png")


def save_fig_04_software_architecture() -> None:
    fig, ax = plt.subplots(figsize=(13.0, 6.8))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 6.8)
    ax.axis("off")

    lanes = [
        (0.20, 1.25, 2.45, 4.85, "Sensor / C2"),
        (3.05, 1.25, 2.65, 4.85, "Perception"),
        (6.00, 1.25, 2.65, 4.85, "Mission logic"),
        (8.95, 1.25, 3.85, 4.85, "Flight output"),
    ]
    for x, y, w, h, label in lanes:
        panel = patches.FancyBboxPatch(
            (x, y),
            w,
            h,
            boxstyle="round,pad=0.02,rounding_size=0.06",
            linewidth=0.8,
            edgecolor="#e2e8f0",
            facecolor="#ffffff",
            zorder=-2,
        )
        ax.add_patch(panel)
        ax.text(x + w / 2, y + h - 0.18, label, ha="center", va="center",
                fontsize=8.8, color=PALETTE["slate"], weight="bold")

    draw_box(ax, (0.45, 4.95), (1.95, 0.74), "Camera", "/image_raw", "#e0f2fe", fs=9.0)
    draw_box(ax, (0.45, 3.65), (1.95, 0.74), "PX4 uORB", "/fmu/out/*", "#fef3c7", fs=9.0)
    draw_box(ax, (0.45, 2.35), (1.95, 0.74), "C2 / Ground", "start / abort", "#fee2e2", fs=9.0)

    draw_box(ax, (3.25, 5.10), (2.25, 0.70), "grid_detector", "intersections", "#dcfce7", fs=8.8)
    draw_box(ax, (3.25, 4.05), (2.25, 0.70), "aruco_tracker", "marker pose", "#ffedd5", fs=8.8)
    draw_box(ax, (3.25, 3.00), (2.25, 0.70), "visual_odometry", "grid-frame pose", "#dbeafe", fs=8.8)
    draw_box(ax, (3.25, 1.95), (2.25, 0.70), "vertiport_detector", "landing error", "#e0f2fe", fs=8.8)

    draw_box(ax, (6.20, 4.35), (2.20, 0.76), "mission_manager", "Phase 1 / Phase 2", "#e0f2fe", fs=8.8)
    draw_box(ax, (6.20, 3.10), (2.20, 0.76), "sprint_planner", "time-cost route", "#f3e8ff", fs=8.8)
    draw_box(ax, (6.20, 1.85), (2.20, 0.76), "safety_gate", "drift / abort clamp", "#fee2e2", fs=8.8)

    draw_box(ax, (9.20, 3.65), (2.15, 0.82), "px4_offboard_control", "20 Hz setpoints", "#dcfce7", fs=8.5)
    draw_box(ax, (11.55, 3.68), (1.10, 0.76), "PX4 FC", "attitude", "#f8fafc", fs=8.8)
    draw_box(ax, (11.55, 2.40), (1.10, 0.76), "Motor", "thrust", "#f8fafc", fs=8.8)

    flow = PALETTE["line"]
    ax.plot([2.40, 2.82], [5.32, 5.32], color=flow, lw=1.3)
    ax.plot([2.82, 2.82], [2.30, 5.45], color=flow, lw=1.3)
    draw_arrow(ax, (2.82, 5.45), (3.25, 5.45), lw=1.3)
    draw_arrow(ax, (2.82, 4.40), (3.25, 4.40), lw=1.3)
    draw_arrow(ax, (2.82, 2.30), (3.25, 2.30), lw=1.3)
    draw_poly_arrow(ax, [(2.40, 4.02), (2.95, 4.02), (2.95, 3.35), (3.25, 3.35)], lw=1.3)
    draw_poly_arrow(ax, [(2.40, 2.72), (5.85, 2.72), (5.85, 4.73), (6.20, 4.73)], lw=1.2)

    draw_arrow(ax, (5.50, 5.45), (6.20, 4.83), lw=1.3)
    draw_arrow(ax, (5.50, 4.40), (6.20, 4.62), lw=1.3)
    draw_arrow(ax, (5.50, 3.35), (6.20, 3.48), lw=1.3)
    draw_arrow(ax, (5.50, 2.30), (6.20, 2.23), lw=1.3)

    draw_arrow(ax, (7.30, 4.35), (7.30, 3.86), lw=1.4)
    draw_arrow(ax, (8.40, 3.48), (9.20, 4.06), lw=1.4)
    draw_poly_arrow(ax, [(8.40, 2.23), (8.78, 2.23), (8.78, 3.93), (9.20, 3.93)], lw=1.2)
    draw_arrow(ax, (11.35, 4.06), (11.55, 4.06), lw=1.4)
    draw_arrow(ax, (12.10, 3.68), (12.10, 3.16), lw=1.4)

    ax.text(6.5, 6.45, "Team-owned ROS2 nodes emphasize perception, planning, and fast Offboard execution",
            fontsize=9.2, ha="center", color=PALETTE["slate"])
    ax.set_title("ROS2/PX4 소프트웨어 아키텍처와 자체 개발 모듈", fontsize=13, weight="bold", color="#111827")
    savefig(REPORT_DIR / "fig_04_software_architecture.png")


def draw_grid(ax):
    for col in range(GRID_COLS):
        x = 2.0 + col * GRID_SPACING
        ax.plot([x, x], [0, -15], color=PALETTE["grid"], lw=0.8, zorder=0)
    for row in range(GRID_ROWS):
        y = 0.0 - row * GRID_SPACING
        ax.plot([2, 26], [y, y], color=PALETTE["grid"], lw=0.8, zorder=0)
    ax.scatter([0], [0], marker="H", s=85, color=PALETTE["green"], edgecolor="white", zorder=5)
    ax.set_xlim(-1, 27.5)
    ax.set_ylim(-16.5, 1.8)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])


def save_fig_09_strategy_comparison() -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 5.2), sharex=True, sharey=True)
    for ax in axes:
        draw_grid(ax)

    left = axes[0]
    line_path = []
    for row in range(GRID_ROWS):
        y = -row * GRID_SPACING
        cols = range(GRID_COLS) if row % 2 == 0 else range(GRID_COLS - 1, -1, -1)
        for col in cols:
            x = 2 + col * GRID_SPACING
            line_path.append((x, y))
    left.plot([p[0] for p in line_path], [p[1] for p in line_path], color=PALETTE["slate"], lw=2.0)
    left.scatter([p[0] for p in line_path[::5]], [p[1] for p in line_path[::5]], s=18, color=PALETTE["orange"], zorder=3)
    left.set_title("Conventional: line following + repeated deceleration", fontsize=11, weight="bold")
    left.text(12, -14.8, "high line-PID effort\nstop/deceleration overhead", ha="center", va="bottom", fontsize=9, color=PALETTE["slate"])

    right = axes[1]
    serp = SERPENTINE_GRID
    right.plot([p[0] for p in serp], [p[1] for p in serp], color=PALETTE["green"], lw=2.5, label="Phase 1")
    current = evaluate_layout(CURRENT_SEED)
    marker_pos = {m.marker_id: m.grid for m in current.markers}
    rescue = [serp[current.all_found_index]] + [marker_pos[mid] for mid in (4, 3, 2, 1)] + [HOME_GRID]
    right.plot([p[0] for p in rescue], [p[1] for p in rescue], color=PALETTE["red"], lw=2.7, linestyle="-", label="Phase 2")
    for mid, pt in marker_pos.items():
        right.scatter(pt[0], pt[1], marker="s", s=95, color=PALETTE["blue"], edgecolor="white", zorder=5)
        right.text(pt[0] + 0.55, pt[1] + 0.55, f"ID {mid}", fontsize=8.5, weight="bold")
    right.set_title("Proposed: intersection sprint + direct rescue", fontsize=11, weight="bold")
    right.text(12, -14.8, "search near grid lines\nrescue ignores grid", ha="center", va="bottom", fontsize=9, color=PALETTE["slate"])
    right.legend(loc="upper center", bbox_to_anchor=(0.5, -0.04), ncol=2, frameon=False, fontsize=8.5)

    fig.suptitle("라인 추종 방식과 소프트웨어 스프린트 전략 비교", fontsize=13, weight="bold")
    savefig(REPORT_DIR / "fig_09_strategy_comparison.png")


def save_fig_10_safety_state_machine() -> None:
    fig, ax = plt.subplots(figsize=(12.5, 5.8))
    ax.set_xlim(0, 12.5)
    ax.set_ylim(0, 5.8)
    ax.axis("off")

    # Box geometry: width=1.6, gap=1.1  →  gap is wide enough for all label text
    BW, BH, GAP = 1.6, 0.9, 1.1
    NX = 1.35                        # NORMAL  left edge
    AX = NX + BW + GAP              # APPROACH left edge  = 4.05
    SX = AX + BW + GAP              # ANTI_SWAY left edge = 6.75
    VX = SX + BW + GAP              # VISION   left edge  = 9.45
    TOP_Y, BOT_Y = 3.3, 1.1
    AY, BY = TOP_Y + BH / 2, BOT_Y + BH / 2   # arrow y: 3.75, 1.55

    states = {
        "NORMAL":   (NX, TOP_Y, "NORMAL\nSPRINT",           "#dcfce7"),
        "APPROACH": (AX, TOP_Y, "APPROACH\nspeed limit",     "#e0f2fe"),
        "ANTI":     (SX, TOP_Y, "ANTI_SWAY\nre-align",       "#fef3c7"),
        "VISION":   (VX, TOP_Y, "VISION_SERVO\nhover / land", "#f3e8ff"),
        "RETURN":   (AX, BOT_Y, "RETURN_HOME\nlow battery",  "#ffedd5"),
        "HOVER":    (SX, BOT_Y, "HOVER_HOLD\nreacquire",     "#e0f2fe"),
        "ABORT":    (VX, BOT_Y, "ABORT_LAND\nimmediate land", "#fee2e2"),
    }
    for x, y, title, color in states.values():
        draw_box(ax, (x, y), (BW, BH), title, "", color=color, fs=9)

    # Top-row arrows — gap=1.1 is wider than any label, so text stays clear of box edges
    draw_arrow(ax, (NX+BW, AY), (AX, AY), text="marker near", text_offset=(0, 0.17))
    draw_arrow(ax, (AX+BW, AY), (SX, AY), text="shake",       text_offset=(0, 0.17))
    draw_arrow(ax, (SX+BW, AY), (VX, AY), text="center lock", text_offset=(0, 0.17))

    # Vertical transitions (top → bottom row)
    AC, SC, VC = AX+BW/2, SX+BW/2, VX+BW/2   # box center x: 4.85, 7.55, 10.25
    draw_arrow(ax, (AC,    TOP_Y), (AC,    BOT_Y+BH), color=PALETTE["orange"])
    draw_arrow(ax, (SC-.2, TOP_Y), (SC-.2, BOT_Y+BH), color=PALETTE["cyan"])
    draw_arrow(ax, (VC,    TOP_Y), (VC,    BOT_Y+BH), color=PALETTE["red"])

    # Bottom-row arrows
    draw_arrow(ax, (AX+BW, BY), (SX, BY), text="home lost", text_offset=(0, 0.17))
    draw_arrow(ax, (SX+BW, BY), (VX, BY), color=PALETTE["red"],
               text="timeout", text_offset=(0, 0.17))

    # Recovery arrow (HOVER → ANTI)
    draw_arrow(ax, (SC+.2, BOT_Y+BH), (SC+.2, TOP_Y), color=PALETTE["green"])

    # Labels on vertical arrows — open inter-row gap, white bg safe
    label_box = dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.88)
    ax.text(AC-.55, 2.55, "low battery", fontsize=8.2, ha="center", color=PALETTE["orange"], bbox=label_box)
    ax.text(SC-.45, 2.55, "drift high",  fontsize=8.2, ha="center", color=PALETTE["cyan"],   bbox=label_box)
    ax.text(SC+.55, 2.55, "reacquired",  fontsize=8.2, ha="center", color=PALETTE["green"],  bbox=label_box)
    ax.text(VC+.55, 2.55, "abort",       fontsize=8.2, ha="center", color=PALETTE["red"],    bbox=label_box)

    # Happy-path annotation
    ax.annotate("", xy=(VX+BW+0.15, 4.52), xytext=(NX-0.1, 4.52),
                arrowprops=dict(arrowstyle="-|>", color="#86efac", lw=1.6))
    ax.text((NX + VX+BW) / 2, 4.66, "정상 임무 흐름 (happy path)",
            fontsize=8.8, ha="center", color="#16a34a", style="italic",
            bbox=dict(boxstyle="round,pad=0.22", facecolor="#f0fdf4",
                      edgecolor="#86efac", linewidth=0.9))

    # Footer notes in a styled box
    ax.text(6.25, 0.50,
            "PX4 failsafe: stabilize/land on Offboard loss, comms loss, or sensor fault"
            "     │     "
            "Mission Manager: mode-specific velocity/jerk envelope and landing gate",
            fontsize=8.0, color=PALETTE["slate"], ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.30", facecolor="#f8fafc",
                      edgecolor="#d0d5dd", linewidth=0.8))

    # Arrow color legend
    legend_elements = [
        Line2D([0], [0], color=PALETTE["line"],   lw=1.8, label="normal flow"),
        Line2D([0], [0], color=PALETTE["orange"], lw=1.8, label="low battery"),
        Line2D([0], [0], color=PALETTE["cyan"],   lw=1.8, label="drift exceeded"),
        Line2D([0], [0], color=PALETTE["green"],  lw=1.8, label="recovery"),
        Line2D([0], [0], color=PALETTE["red"],    lw=1.8, label="abort / timeout"),
    ]
    ax.legend(handles=legend_elements, loc="upper left",
              bbox_to_anchor=(0.005, 0.995), bbox_transform=ax.transAxes,
              fontsize=7.8, frameon=True, fancybox=False, edgecolor="#d0d5dd")

    ax.set_title("고속 스프린트 운용 안전 상태 전이", fontsize=13, weight="bold", color="#111827")
    savefig(REPORT_DIR / "fig_10_safety_state_machine.png")


def save_fig_11_sweep_results(results: list[SweepResult]) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8))
    colors = {"clustered": PALETTE["orange"], "mixed": PALETTE["blue"], "spread": PALETTE["green"]}
    labels_ko = {"clustered": "clustered", "mixed": "mixed", "spread": "spread"}
    groups = grouped(results)
    current = evaluate_layout(CURRENT_SEED)

    ax = axes[0]
    for name in ("clustered", "mixed", "spread"):
        vals = groups[name]
        ax.scatter(
            [v.min_pair_distance_m for v in vals],
            [v.total_time_s for v in vals],
            s=18,
            alpha=0.55,
            color=colors[name],
            label=labels_ko[name],
            edgecolors="none",
        )
    ax.scatter(
        [current.min_pair_distance_m],
        [current.total_time_s],
        s=120,
        marker="*",
        color=PALETTE["red"],
        edgecolor="white",
        linewidth=1.0,
        label="current seed 310",
        zorder=5,
    )
    ax.set_xlabel("minimum pairwise marker distance (m)")
    ax.set_ylabel("modeled mission time (s)")
    style_axes(ax)
    ax.legend()
    ax.set_title("(a) Layout spread vs. modeled time")

    ax = axes[1]
    data = [[v.total_time_s for v in groups[name]] for name in ("clustered", "mixed", "spread")]
    bp = ax.boxplot(data, tick_labels=["clustered", "mixed", "spread"], patch_artist=True, showfliers=False)
    for patch, name in zip(bp["boxes"], ("clustered", "mixed", "spread")):
        patch.set_facecolor(colors[name])
        patch.set_alpha(0.55)
        patch.set_edgecolor(PALETTE["line"])
    for med in bp["medians"]:
        med.set_color("#111827")
        med.set_linewidth(1.8)
    ax.axhline(current.total_time_s, color=PALETTE["red"], lw=1.4, linestyle="--", label="current seed")
    ax.set_ylabel("modeled mission time (s)")
    style_axes(ax)
    ax.legend()
    ax.set_title("(b) Time distribution by layout class")

    ax = axes[2]
    bins = range(0, 56, 5)
    for name in ("clustered", "mixed", "spread"):
        ax.hist(
            [v.all_found_index + 1 for v in groups[name]],
            bins=bins,
            alpha=0.45,
            color=colors[name],
            label=labels_ko[name],
        )
    ax.axvline(current.all_found_index + 1, color=PALETTE["red"], lw=1.8, linestyle="--", label="current seed")
    ax.set_xlabel("all markers found at intersection index / 54")
    ax.set_ylabel("number of cases")
    style_axes(ax)
    ax.legend()
    ax.set_title("(c) Search termination index")

    fig.suptitle("ArUco 랜덤 배치 Sweep 결과", fontsize=13, weight="bold")
    savefig(REPORT_DIR / "fig_11_marker_sweep_results.png")


def representative_cases(results: list[SweepResult]) -> list[tuple[str, SweepResult]]:
    groups = grouped(results)
    clustered_pool = [r for r in groups["clustered"] if r.span_x_m >= 6.0 and r.span_y_m >= 3.0]
    clustered = min(clustered_pool or groups["clustered"], key=lambda r: (r.min_pair_distance_m, r.total_time_s))
    mixed_target = median([r.min_pair_distance_m for r in groups["mixed"]])
    mixed = min(groups["mixed"], key=lambda r: abs(r.min_pair_distance_m - mixed_target))
    current = evaluate_layout(CURRENT_SEED)
    return [("Clustered layout", clustered), ("Intermediate layout", mixed), ("Current Gazebo seed 310", current)]


def save_fig_12_layout_cases(results: list[SweepResult]) -> None:
    cases = representative_cases(results)
    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.05), sharex=True, sharey=True)
    for ax, (title, result) in zip(axes, cases):
        draw_grid(ax)
        marker_pos = {m.marker_id: m.grid for m in result.markers}
        serp = SERPENTINE_GRID[: result.all_found_index + 1]
        ax.plot([p[0] for p in serp], [p[1] for p in serp], color=PALETTE["green"], lw=2.0, alpha=0.85)
        rescue = [serp[-1]] + [marker_pos[mid] for mid in (4, 3, 2, 1)] + [HOME_GRID]
        ax.plot([p[0] for p in rescue], [p[1] for p in rescue], color=PALETTE["red"], lw=2.2, alpha=0.85)
        for mid, pt in marker_pos.items():
            ax.scatter(pt[0], pt[1], s=110, marker="s", color=PALETTE["blue"], edgecolor="white", linewidth=1.0, zorder=5)
            offsets = {1: (8, 8), 2: (8, -14), 3: (8, 8), 4: (8, 8)}
            ax.annotate(
                f"ID {mid}",
                xy=pt,
                xytext=offsets[mid],
                textcoords="offset points",
                fontsize=8.2,
                weight="bold",
                color="#111827",
                bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.9),
                arrowprops=dict(arrowstyle="-", color=PALETTE["slate"], lw=0.7, alpha=0.7),
                zorder=6,
            )
        ax.set_title(
            f"{title}\nseed {result.seed}, min {result.min_pair_distance_m:.1f}m, total {result.total_time_s:.1f}s",
            fontsize=10,
            weight="bold",
        )

    handles = [
        Line2D([0], [0], color=PALETTE["green"], lw=2.2, label="Phase 1 search until all found"),
        Line2D([0], [0], color=PALETTE["red"], lw=2.2, label="Phase 2 direct rescue/return"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor=PALETTE["blue"], markeredgecolor="white", markersize=8, label="ArUco marker"),
    ]
    fig.subplots_adjust(top=0.82, bottom=0.18, wspace=0.18)
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, fontsize=9)
    fig.suptitle("대표 ArUco 배치별 탐색 종료와 구조 스프린트", fontsize=13, weight="bold")
    savefig(REPORT_DIR / "fig_12_marker_layout_cases.png")


def main() -> None:
    ensure_dir()
    results = run_sweep()
    speed_results = phase1_speed_sweep()
    write_sweep_csv(results)
    write_sweep_summary(results)
    write_phase1_speed_sweep_csv(speed_results)
    write_phase1_speed_sweep_summary(speed_results)
    save_fig_02_mission_flow()
    save_fig_03_optical_flow_snap()
    save_fig_04_software_architecture()
    save_fig_09_strategy_comparison()
    save_fig_10_safety_state_machine()
    save_fig_11_sweep_results(results)
    save_fig_12_layout_cases(results)
    save_fig_13_phase1_speed_tradeoff(speed_results)
    print(f"Generated report figures and sweep artifacts in {REPORT_DIR}")


if __name__ == "__main__":
    main()
