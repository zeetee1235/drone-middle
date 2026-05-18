#!/usr/bin/env python3
"""Generate the Phase 1 search-path report figure.

This figure is intentionally a mission-design figure, not a partial rosbag
trace. It shows the current ArUco layout, candidate-detection zones, and
the 54-intersection serpentine search route used by the technical plan.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

KO_FONT_PATH = Path("/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf")
if KO_FONT_PATH.exists():
    fm.fontManager.addfont(str(KO_FONT_PATH))
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=str(KO_FONT_PATH)).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False


START_X = 2.0
START_Y = 19.0
GRID_SPACING = 3.0
GRID_COL_OFFSET = 2.0
GRID_ROWS = 6
GRID_COLS = 9

MARKERS_WORLD = {
    1: (19.0, 19.0),
    2: (4.0, 16.0),
    3: (10.0, 4.0),
    4: (25.0, 4.0),
}
MARKERS = {
    mid: (x - START_X, y - START_Y)
    for mid, (x, y) in MARKERS_WORLD.items()
}

LABEL_OFFSETS = {
    1: (1.10, 1.05),
    2: (-1.25, 1.05),
    3: (-1.10, 1.05),
    4: (1.10, 1.05),
}


def build_serpentine() -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for row in range(GRID_ROWS):
        y = -row * GRID_SPACING
        cols = range(GRID_COLS) if row % 2 == 0 else range(GRID_COLS - 1, -1, -1)
        for col in cols:
            pts.append((GRID_COL_OFFSET + col * GRID_SPACING, y))
    return pts


def add_arrows(ax, pts: list[tuple[float, float]], color: str) -> None:
    step = max(1, len(pts) // 14)
    for i in range(step // 2, len(pts) - step, step):
        x0, y0 = pts[i]
        x1, y1 = pts[i + step]
        if (x1 - x0) ** 2 + (y1 - y0) ** 2 < 0.04:
            continue
        ax.annotate(
            "",
            xy=(x1, y1),
            xytext=(x0, y0),
            arrowprops=dict(
                arrowstyle="-|>",
                color=color,
                lw=0.75,
                mutation_scale=7,
                shrinkA=0,
                shrinkB=0,
            ),
            zorder=6,
        )


def draw_marker(ax, marker_id: int, x: float, y: float, colors: dict[str, str]) -> None:
    ax.scatter(
        [x],
        [y],
        marker="s",
        s=140,
        facecolor="white",
        edgecolor="#7a3f00",
        linewidth=1.0,
        zorder=12,
    )
    ax.scatter(
        [x],
        [y],
        marker="s",
        s=78,
        facecolor=colors["marker"],
        edgecolor="white",
        linewidth=0.8,
        zorder=13,
    )

    dx, dy = LABEL_OFFSETS[marker_id]
    tx, ty = x + dx, y + dy
    ax.annotate(
        f"M{marker_id}",
        xy=(x, y),
        xytext=(tx, ty),
        ha="center",
        va="center",
        fontsize=8,
        color="#7a3f00",
        fontweight="bold",
        zorder=15,
        bbox=dict(
            boxstyle="round,pad=0.22",
            facecolor="white",
            edgecolor=colors["marker"],
            linewidth=0.7,
            alpha=0.96,
        ),
        arrowprops=dict(
            arrowstyle="-",
            color=colors["marker"],
            lw=0.7,
            shrinkA=4,
            shrinkB=6,
            alpha=0.9,
        ),
    )


def plot(out: Path) -> None:
    colors = {
        "search": "#2166ac",
        "marker": "#e08214",
        "detect": "#b35806",
        "vertiport": "#252525",
        "safe_fill": "#f7f7f7",
        "safe_edge": "#bbbbbb",
        "miss_fill": "#e8f0f8",
        "miss_edge": "#5b8db8",
        "grid_line": "#c8d8e8",
        "wp_dot": "#aec7e0",
    }

    serpentine = build_serpentine()
    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    ax.add_patch(Rectangle((-2, -19), 32, 23, linewidth=0.8,
                           edgecolor=colors["safe_edge"], facecolor=colors["safe_fill"], zorder=0))
    ax.add_patch(Rectangle((2, -15), 24, 15, linewidth=1.0,
                           edgecolor=colors["miss_edge"], facecolor=colors["miss_fill"], zorder=1))

    for y in [0, -3, -6, -9, -12, -15]:
        ax.plot([2, 26], [y, y], color=colors["grid_line"], lw=0.55, zorder=2)
    for x in [2, 5, 8, 11, 14, 17, 20, 23, 26]:
        ax.plot([x, x], [-15, 0], color=colors["grid_line"], lw=0.55, zorder=2)

    for x, y in serpentine:
        ax.plot(x, y, ".", color=colors["wp_dot"], ms=3.5, zorder=3, mew=0)

    xs, ys = zip(*serpentine)
    ax.plot(xs, ys, color=colors["search"], lw=1.6, zorder=5,
            solid_capstyle="round", solid_joinstyle="round")
    add_arrows(ax, serpentine, colors["search"])

    for x, y in MARKERS.values():
        ring = plt.Circle((x, y), 1.5, color=colors["detect"], fill=False,
                          lw=0.85, ls="--", alpha=0.48, zorder=4)
        ring.set_path_effects([pe.withStroke(linewidth=2.2, foreground="white", alpha=0.7)])
        ax.add_patch(ring)

    for marker_id, (x, y) in MARKERS.items():
        draw_marker(ax, marker_id, x, y, colors)

    ax.add_patch(plt.Circle((0, 0), 1.5, color=colors["vertiport"], fill=False,
                            lw=1.0, alpha=0.45, zorder=4))
    ax.plot(0, 0, "o", color=colors["vertiport"], ms=9, zorder=12)
    ax.plot(0, 0, "+", color="white", ms=7, mew=1.2, zorder=13)
    ax.text(1.8, 0.3, "Vertiport", color=colors["vertiport"],
            fontsize=7.5, va="center", style="italic")

    ax.text(14, -16.0, "Mission Area  (24 m x 15 m)", color=colors["miss_edge"],
            ha="center", va="top", fontsize=7.5, style="italic")
    ax.text(-1.5, -9.0, "Safe\nArea", color=colors["safe_edge"],
            ha="center", va="center", fontsize=7, style="italic", rotation=90)

    ax.set_title(
        "Phase 1 탐색 경로: 교차점 스프린트와 ArUco 후보 검출",
        fontsize=9.5,
        pad=6,
    )
    ax.set_xlabel("$x$ (m, east from vertiport)", fontsize=9)
    ax.set_ylabel("$y$ (m, north from vertiport)", fontsize=9)
    ax.set_xlim(-3, 30)
    ax.set_ylim(-20, 4.5)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=8, direction="in", top=True, right=True)
    ax.set_xticks(range(0, 30, 3))
    ax.set_yticks(range(-15, 4, 3))

    legend = [
        Line2D([0], [0], color=colors["search"], lw=1.6, label="54-intersection search"),
        Line2D([0], [0], marker="s", color="#7a3f00", mfc=colors["marker"],
               ms=7, mew=0.8, label="ArUco marker"),
        Line2D([0], [0], color=colors["detect"], lw=0.85, ls="--",
               label="candidate detection zone"),
        Line2D([0], [0], marker="o", color="none", mfc=colors["vertiport"],
               ms=7, mew=0, label="Vertiport"),
    ]
    ax.legend(
        handles=legend,
        loc="center left",
        bbox_to_anchor=(1.01, 0.5),
        fontsize=7.2,
        framealpha=0.92,
        edgecolor="0.6",
        fancybox=False,
    )

    plt.tight_layout(rect=[0, 0, 0.82, 1])
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path,
                        default=Path("reports/fig_05_gazebo_search_path.png"))
    args = parser.parse_args()
    plot(args.out)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
