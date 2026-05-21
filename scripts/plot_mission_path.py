#!/usr/bin/env python3
"""
plot_mission_path.py — 전체 임무 비행 경로 시각화

grid frame 좌표(vertiport=원점)를 matplotlib으로 그린다.
"""

import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
_KO_FONT_PATH = "/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf"
_ko_prop = fm.FontProperties(fname=_KO_FONT_PATH)
_KO_FONT = _ko_prop.get_name()
fm.fontManager.addfont(_KO_FONT_PATH)
matplotlib.rcParams["font.family"] = _KO_FONT
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.lines import Line2D

# ── 세계 상수 ──────────────────────────────────────────────────────────────────
START_X, START_Y = 2.0, 19.0
CRUISE_Z  = 2.0
GRID_SPACING    = 3.0
GRID_COL_OFFSET = 2.0
GRID_ROWS = 6
GRID_COLS = 9
V_MAX     = 10.0
WAYPOINT_TOL   = 1.5
MARKER_DETECT_R = 1.5
HOVER_DUR = 3.0
DT = 0.05   # 더 촘촘하게

MARKERS = {
    1: {"gx": 17.0, "gy":  0.0},
    2: {"gx":  2.0, "gy": -3.0},
    3: {"gx":  8.0, "gy":-15.0},
    4: {"gx": 23.0, "gy":-15.0},
}

MARKER_LABEL_OFFSETS = {
    1: (1.10, 1.05),
    2: (-1.25, 1.05),
    3: (-1.10, 1.05),
    4: (1.10, 1.05),
}

def build_serpentine():
    pts = []
    for row in range(GRID_ROWS):
        y = -row * GRID_SPACING
        cols = range(GRID_COLS) if row % 2 == 0 else range(GRID_COLS-1, -1, -1)
        for col in cols:
            pts.append((GRID_COL_OFFSET + col * GRID_SPACING, y))
    return pts

SERPENTINE = build_serpentine()

def dist(ax, ay, bx, by):
    return math.sqrt((ax-bx)**2 + (ay-by)**2)

def move_toward(cx, cy, tx, ty, speed, dt):
    d = dist(cx, cy, tx, ty)
    if d < 1e-6:
        return cx, cy
    step = min(speed * dt, d)
    return cx + (tx-cx)/d*step, cy + (ty-cy)/d*step

# ── 시뮬레이션 (경로 좌표 수집) ──────────────────────────────────────────────
def simulate():
    t, gx, gy = 0.0, 0.0, 0.0
    phase = "INIT"
    confirmed = {}
    detected  = set()
    visited   = []
    rescue_order = []
    serpentine_idx = 0
    hover_start = None
    approach_target = None
    rescue_target   = None
    rescue_idx      = 0
    home_init_start = None

    # 경로 세그먼트별 포인트 수집
    segments = {
        "takeoff":   [],
        "search":    [],
        "approach":  [],
        "hover":     [],
        "rescue":    [],
        "return":    [],
        "land":      [],
    }
    events = []   # (t, gx, gy, label)
    cur_seg = "takeoff"

    def rec(seg=None):
        nonlocal cur_seg
        if seg:
            cur_seg = seg
        segments[cur_seg].append((gx, gy))

    tick = 0
    while phase != "LANDED" and t < 600.0:
        t = tick * DT
        tick += 1

        if phase == "INIT":
            phase = "TAKEOFF"

        elif phase == "TAKEOFF":
            rec("takeoff")
            if t >= CRUISE_Z / 1.7:
                phase = "HOME_INIT"
                home_init_start = t

        elif phase == "HOME_INIT":
            rec("takeoff")
            if t - home_init_start >= 1.0:
                phase = "GRID_SEARCH"
                serpentine_idx = 0

        elif phase == "GRID_SEARCH":
            if len(confirmed) >= 4:
                phase = "RESCUE_ROUTE_PLAN"
                continue
            if serpentine_idx < len(SERPENTINE):
                tx, ty = SERPENTINE[serpentine_idx]
                gx, gy = move_toward(gx, gy, tx, ty, V_MAX, DT)
                if dist(gx, gy, tx, ty) < WAYPOINT_TOL:
                    serpentine_idx += 1
            else:
                phase = "RETURN_HOME"
                continue
            rec("search")
            for mid, mpos in MARKERS.items():
                if mid in confirmed or mid in detected:
                    continue
                if dist(gx, gy, mpos["gx"], mpos["gy"]) < MARKER_DETECT_R:
                    detected.add(mid)
                    approach_target = (mpos["gx"], mpos["gy"], mid)
                    events.append((t, gx, gy, f"감지 #{mid}"))
                    phase = "MARKER_APPROACH"
                    break

        elif phase == "MARKER_APPROACH":
            if approach_target:
                tx, ty, _ = approach_target
                gx, gy = move_toward(gx, gy, tx, ty, V_MAX * 0.5, DT)
                if dist(gx, gy, tx, ty) < 0.3:
                    phase = "ANTI_SWAY"
            rec("approach")

        elif phase == "ANTI_SWAY":
            rec("hover")
            if hover_start is None:
                hover_start = t
            if t - hover_start >= 0.5:
                hover_start = None
                phase = "HOVER_CONFIRM"

        elif phase == "HOVER_CONFIRM":
            rec("hover")
            if hover_start is None:
                hover_start = t
            if t - hover_start >= HOVER_DUR:
                hover_start = None
                phase = "MARKER_SAVE"

        elif phase == "MARKER_SAVE":
            if approach_target:
                mid = approach_target[2]
                confirmed[mid] = {"gx": approach_target[0], "gy": approach_target[1]}
                events.append((t, gx, gy, f"저장 #{mid}"))
                approach_target = None
            phase = "GRID_SEARCH" if len(confirmed) < 4 else "RESCUE_ROUTE_PLAN"

        elif phase == "RESCUE_ROUTE_PLAN":
            rescue_order = sorted(confirmed.keys(), reverse=True)
            rescue_idx = 0
            phase = "RESCUE_VISIT"

        elif phase == "RESCUE_VISIT":
            if rescue_idx >= len(rescue_order):
                phase = "RETURN_HOME"
                continue
            mid = rescue_order[rescue_idx]
            mpos = confirmed[mid]
            if rescue_target is None or rescue_target[2] != mid:
                rescue_target = (mpos["gx"], mpos["gy"], mid)
                hover_start = None
            tx, ty, _ = rescue_target
            d = dist(gx, gy, tx, ty)
            if d > 0.3:
                gx, gy = move_toward(gx, gy, tx, ty, V_MAX, DT)
            else:
                if hover_start is None:
                    hover_start = t
                    events.append((t, gx, gy, f"구조 #{mid}"))
                if t - hover_start >= HOVER_DUR:
                    visited.append(mid)
                    rescue_idx += 1
                    hover_start = None
                    rescue_target = None
                    if rescue_idx >= len(rescue_order):
                        phase = "RETURN_HOME"
            rec("rescue")

        elif phase == "RETURN_HOME":
            gx, gy = move_toward(gx, gy, 0.0, 0.0, V_MAX, DT)
            rec("return")
            if dist(gx, gy, 0.0, 0.0) < WAYPOINT_TOL:
                phase = "VERTIPORT_ACQUIRE"

        elif phase == "VERTIPORT_ACQUIRE":
            gx, gy = move_toward(gx, gy, 0.0, 0.0, V_MAX * 0.3, DT)
            rec("land")
            if dist(gx, gy, 0.0, 0.0) < 0.3:
                phase = "VISION_SERVO_LAND"
                hover_start = t

        elif phase == "VISION_SERVO_LAND":
            rec("land")
            if hover_start and t - hover_start >= 4.0:
                phase = "LANDED"

    return segments, events, rescue_order

# ── 그리기 (논문 스타일) ──────────────────────────────────────────────────────
import os
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev

segments, events, rescue_order = simulate()

# ── 경로 스무딩 ───────────────────────────────────────────────────────────────
def smooth_seg(pts, sigma=3.0, upsample=4):
    """
    Gaussian smoothing + spline upsample.
    sigma  : 스무딩 강도 (높을수록 코너가 더 둥글게)
    upsample: 출력 포인트 배율
    """
    if len(pts) < 4:
        return pts
    arr = np.array(pts)
    xs = gaussian_filter1d(arr[:, 0], sigma=sigma)
    ys = gaussian_filter1d(arr[:, 1], sigma=sigma)
    # 스플라인으로 업샘플링 → 선이 더 매끄럽게 보임
    try:
        tck, u = splprep([xs, ys], s=0, k=3)
        u_new = np.linspace(0, 1, len(pts) * upsample)
        xs2, ys2 = splev(u_new, tck)
        return list(zip(xs2, ys2))
    except Exception:
        return list(zip(xs, ys))


def split_runs(pts, max_gap=1.0):
    """Split repeated sub-state samples so disconnected marker events are not joined."""
    runs = []
    current = []
    for point in pts:
        if current and dist(current[-1][0], current[-1][1], point[0], point[1]) > max_gap:
            if len(current) >= 2:
                runs.append(current)
            current = []
        current.append(point)
    if len(current) >= 2:
        runs.append(current)
    return runs


approach_runs = [
    smooth_seg(run, sigma=1.2, upsample=4)
    for run in split_runs(segments["approach"], max_gap=1.0)
]
hover_runs = split_runs(segments["hover"], max_gap=1.0)
hover_centers = [
    (float(np.mean([p[0] for p in run])), float(np.mean([p[1] for p in run])))
    for run in hover_runs
]

# 각 세그먼트별 최적 sigma 설정
#  - search: 180° 전환점이 많아 강하게 스무딩
#  - rescue/return: 직선에 가까우므로 약하게 (끝점 유지 중요)
seg_sigma = {
    "search":   5.0,
    "rescue":   4.0,
    "return":   4.0,
    "land":     2.0,
}
for key, sigma in seg_sigma.items():
    if key in segments and len(segments[key]) >= 4:
        segments[key] = smooth_seg(segments[key], sigma=sigma, upsample=5)

# ── rcParams — IEEE/ACM 논문 스타일 ───────────────────────────────────────────
plt.rcParams.update({
    "axes.linewidth":    0.8,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "xtick.minor.width": 0.4,
    "ytick.minor.width": 0.4,
    "xtick.direction":   "in",
    "ytick.direction":   "in",
    "xtick.top":         True,
    "ytick.right":       True,
    "xtick.major.size":  3.5,
    "ytick.major.size":  3.5,
    "xtick.minor.size":  2.0,
    "ytick.minor.size":  2.0,
    "axes.grid":         False,
    "legend.frameon":    True,
    "legend.edgecolor":  "0.75",
    "legend.fancybox":   False,
})

# ── 색상 팔레트 (색약 배려 + 흑백 인쇄 구분 가능) ────────────────────────────
C = {
    "search":   "#2166ac",   # 짙은 파랑
    "approach": "#f4a582",   # 살구
    "hover":    "#d6604d",   # 벽돌빨강
    "rescue":   "#762a83",   # 자주
    "return":   "#1b7837",   # 짙은 초록
    "land":     "#74c476",   # 연초록
    "marker":   "#e08214",   # 주황
    "detect":   "#b35806",   # 진주황
    "vertiport":"#252525",   # 검정
    "safe_fill":"#f7f7f7",
    "safe_edge":"#bbbbbb",
    "miss_fill":"#e8f0f8",
    "miss_edge":"#5b8db8",
    "grid_line":"#c8d8e8",
    "wp_dot":   "#aec7e0",
}

import matplotlib.ticker

SEGMENT_STYLE = {
    "takeoff":   (C["vertiport"], 1.3, "-",  7),
    "search":    (C["search"],    1.45, "-",  7),
    "approach":  (C["approach"],  1.35, "-",  8),
    "hover":     (C["hover"],     1.25, "-",  8),
    "rescue":    (C["rescue"],    1.65, "-",  9),
    "return":    (C["return"],    1.45, "--", 9),
    "land":      (C["land"],      1.25, ":",  8),
}


def setup_map(ax, title):
    ax.set_facecolor("white")
    safe = mpatches.Rectangle(
        (-2, -19), 32, 23,
        linewidth=0.75, edgecolor=C["safe_edge"], facecolor=C["safe_fill"], zorder=0
    )
    ax.add_patch(safe)

    miss = mpatches.Rectangle(
        (2, -15), 24, 15,
        linewidth=0.95, edgecolor=C["miss_edge"], facecolor=C["miss_fill"],
        linestyle="-", zorder=1
    )
    ax.add_patch(miss)

    for row in range(GRID_ROWS):
        y = -row * GRID_SPACING
        ax.plot([2, 26], [y, y], color=C["grid_line"], lw=0.45, zorder=2)
    for col in range(GRID_COLS):
        x = GRID_COL_OFFSET + col * GRID_SPACING
        ax.plot([x, x], [-15, 0], color=C["grid_line"], lw=0.45, zorder=2)

    for wx, wy in SERPENTINE:
        ax.plot(wx, wy, ".", color=C["wp_dot"], ms=2.8, zorder=3, mew=0)

    for mid, mpos in MARKERS.items():
        mx, my = mpos["gx"], mpos["gy"]
        ax.scatter([mx], [my], marker="s", s=80, facecolor="white",
                   edgecolor="#7a3f00", linewidth=0.85, zorder=12)
        ax.scatter([mx], [my], marker="s", s=42, facecolor=C["marker"],
                   edgecolor="white", linewidth=0.65, zorder=13)
        dx, dy = MARKER_LABEL_OFFSETS[mid]
        ax.annotate(
            f"M{mid}",
            xy=(mx, my),
            xytext=(mx + dx * 0.72, my + dy * 0.72),
            ha="center",
            va="center",
            fontsize=6.6,
            color="#7a3f00",
            fontweight="bold",
            zorder=15,
            bbox=dict(
                boxstyle="round,pad=0.18",
                facecolor="white",
                edgecolor=C["marker"],
                linewidth=0.55,
                alpha=0.96,
            ),
            arrowprops=dict(
                arrowstyle="-",
                color=C["marker"],
                lw=0.55,
                shrinkA=3,
                shrinkB=4,
                alpha=0.85,
            ),
        )

    vp_ring = plt.Circle((0, 0), 1.5, color=C["vertiport"], fill=False,
                         lw=0.85, ls="-", zorder=4, alpha=0.38)
    ax.add_patch(vp_ring)
    ax.plot(0, 0, "o", color=C["vertiport"], ms=7,
            mec=C["vertiport"], mew=0.9, zorder=12)
    ax.plot(0, 0, "+", color="white", ms=5.6, mew=1.0, zorder=13)
    ax.text(1.35, 0.35, "Vertiport", color=C["vertiport"],
            fontsize=6.5, va="center", style="italic")

    ax.text(14, -16.0, "Mission Area  (24 m x 15 m)",
            color=C["miss_edge"], ha="center", va="top", fontsize=6.5,
            style="italic")
    ax.text(-1.5, -9.0, "Safe\nArea", color=C["safe_edge"],
            ha="center", va="center", fontsize=6.2, style="italic", rotation=90)

    ax.set_xlim(-3, 29)
    ax.set_ylim(-20, 4.5)
    ax.set_aspect("equal")
    ax.set_xlabel("$x$ (m, east)", fontsize=7.6)
    ax.set_ylabel("$y$ (m, north)", fontsize=7.6)
    ax.tick_params(labelsize=6.8)
    ax.set_xticks(range(0, 30, 6))
    ax.set_yticks(range(-18, 5, 6))
    ax.xaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(3))
    ax.yaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(3))
    ax.set_title(title, fontsize=8.8, pad=5, fontweight="bold")


def plot_segment(ax, key, alpha=1.0):
    if key == "approach":
        color, lw, ls, z = SEGMENT_STYLE[key]
        for pts in approach_runs:
            if len(pts) < 2:
                continue
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=color, lw=lw, ls=ls, alpha=alpha, zorder=z,
                    solid_capstyle="round", solid_joinstyle="round")
        return
    if key == "hover":
        color, _, _, z = SEGMENT_STYLE[key]
        for x, y in hover_centers:
            ax.scatter([x], [y], marker="o", s=92, facecolor="none",
                       edgecolor=color, linewidth=1.4, zorder=z + 5)
            ax.plot(x, y, marker="+", color=color, ms=6.0, mew=1.2, zorder=z + 6)
        return
    pts = segments.get(key, [])
    if len(pts) < 2:
        return
    color, lw, ls, z = SEGMENT_STYLE[key]
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, alpha=alpha, zorder=z,
            solid_capstyle="round", solid_joinstyle="round")


def add_arrows(ax, pts_list, color, n=8, ms=6):
    pts = pts_list
    if len(pts) < 3:
        return
    step = max(1, len(pts) // n)
    for i in range(step // 2, len(pts) - step, step):
        x0, y0 = pts[i]
        x1, y1 = pts[i + step]
        dx, dy = x1 - x0, y1 - y0
        if dx * dx + dy * dy < 0.04:
            continue
        ax.annotate(
            "",
            xy=(x1, y1),
            xytext=(x0, y0),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=0.58, mutation_scale=ms),
            zorder=11,
        )


def draw_detection_events(ax):
    for t_ev, gx_ev, gy_ev, label in events:
        if "감지" in label:
            ax.plot(gx_ev, gy_ev, "^", color=C["detect"],
                    ms=5.2, mec="white", mew=0.55, zorder=14)


def draw_rescue_events(ax):
    rescue_seq = {mid: i + 1 for i, mid in enumerate(rescue_order)}
    for t_ev, gx_ev, gy_ev, label in events:
        if "구조" in label:
            mid = int(label.split("#")[1])
            seq = rescue_seq[mid]
            ax.plot(gx_ev, gy_ev, "D", color=C["rescue"],
                    ms=6.2, mec="white", mew=0.7, zorder=14, alpha=0.9)
            ax.text(gx_ev, gy_ev, str(seq), color="white",
                    fontsize=5.8, ha="center", va="center",
                    fontweight="bold", zorder=15)


fig, axes = plt.subplots(2, 2, figsize=(11.2, 8.0))
fig.patch.set_facecolor("white")

panel_specs = [
    ("Phase 0  Takeoff / Home Init", ("takeoff",)),
    ("Phase 1  Search + Continuous ArUco", ("search", "approach", "hover")),
    ("Phase 2  Reverse-ID Rescue Sprint", ("rescue",)),
    ("Phase 3  Return / Vision-Servo Land", ("return", "land")),
]

for ax, (title, keys) in zip(axes.flat, panel_specs):
    setup_map(ax, title)
    for key in keys:
        plot_segment(ax, key)
    if "takeoff" in keys:
        ax.plot(0, 0, marker="o", ms=9, mfc="white", mec=C["vertiport"],
                mew=1.4, zorder=16)
        ax.text(2.4, 2.2, "2 m takeoff\nhome pose init",
                fontsize=6.8, color=C["vertiport"], ha="left", va="center",
                bbox=dict(facecolor="white", edgecolor=C["safe_edge"], linewidth=0.55, alpha=0.92))
    if "search" in keys:
        add_arrows(ax, segments["search"], C["search"], n=12, ms=5.5)
        draw_detection_events(ax)
    if "rescue" in keys:
        add_arrows(ax, segments["rescue"], C["rescue"], n=5, ms=6.2)
        draw_rescue_events(ax)
    if "return" in keys:
        add_arrows(ax, segments["return"], C["return"], n=3, ms=6.2)

legend_elems = [
    Line2D([0], [0], color=C["search"],   lw=1.5,           label="Grid search"),
    Line2D([0], [0], color=C["approach"], lw=1.4,           label="Candidate approach"),
    Line2D([0], [0], marker="o", color="none", mfc="none", mec=C["hover"],
           ms=6.5, mew=1.3, label="Hover confirm point"),
    Line2D([0], [0], color=C["rescue"],   lw=1.8,           label="Rescue visit"),
    Line2D([0], [0], color=C["return"],   lw=1.5, ls="--",  label="Return home"),
    Line2D([0], [0], color=C["land"],     lw=1.2, ls=":",   label="Landing"),
    Line2D([0], [0], marker="s", color="none", mfc=C["marker"],
           ms=6, mew=0, label="ArUco marker"),
    Line2D([0], [0], marker="^", color="none", mfc=C["detect"],
           ms=5.5, mew=0, label="Detection point"),
    Line2D([0], [0], marker="D", color="none", mfc=C["rescue"],
           ms=6, mew=0, label="Rescue waypoint (order)"),
    Line2D([0], [0], marker="o", color="none", mfc=C["vertiport"],
           ms=6, mew=0, label="Vertiport"),
]
fig.legend(
    handles=legend_elems,
    loc="lower center",
    bbox_to_anchor=(0.5, 0.015),
    fontsize=7.0,
    ncol=5,
    handlelength=2.0,
    columnspacing=1.0,
    framealpha=0.95,
    edgecolor="0.65",
)

fig.suptitle("전체 임무 비행 경로: Phase별 분리 표시", fontsize=11.2, fontweight="bold", y=0.98)
plt.tight_layout(rect=[0.02, 0.09, 0.98, 0.955])
os.makedirs("/home/dev/drone_middle/reports", exist_ok=True)
outputs = [
    "/home/dev/drone_middle/reports/mission_path.png",
    "/home/dev/drone_middle/reports/fig_06_gazebo_mission_path.png",
]
for out in outputs:
    plt.savefig(out, dpi=200, bbox_inches="tight", facecolor="white")
    print(f"저장: {out}")
