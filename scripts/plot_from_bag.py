#!/usr/bin/env python3
"""
plot_from_bag.py — real rosbag data → paper-quality mission path figure

Usage:
    python3 scripts/plot_from_bag.py <bag_dir>  [--out reports/mission_path_real.png]

Reads:
    /localization/pose_grid  (PoseStamped)
    /mission/state           (String)
"""

import argparse
import os
import sys
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm

_KO_FONT_PATH = "/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf"
if os.path.exists(_KO_FONT_PATH):
    _ko_prop = fm.FontProperties(fname=_KO_FONT_PATH)
    _KO_FONT = _ko_prop.get_name()
    fm.fontManager.addfont(_KO_FONT_PATH)
    matplotlib.rcParams["font.family"] = _KO_FONT

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.ticker
from matplotlib.lines import Line2D
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    sys.exit("pip install rosbags")

# ── 상수 ──────────────────────────────────────────────────────────────────────
START_X, START_Y = 2.0, 19.0
GRID_SPACING    = 3.0
GRID_COL_OFFSET = 2.0
GRID_ROWS = 6
GRID_COLS = 9

MARKERS = {
    1: {"gx": 19.0 - START_X, "gy": 19.0 - START_Y},   # grid (17, 0)
    2: {"gx":  4.0 - START_X, "gy": 16.0 - START_Y},   # grid (2, -3)
    3: {"gx": 10.0 - START_X, "gy":  4.0 - START_Y},   # grid (8, -15)
    4: {"gx": 25.0 - START_X, "gy":  4.0 - START_Y},   # grid (23, -15)
}

MARKER_LABEL_OFFSETS = {
    1: (1.10, 1.05),
    2: (-1.25, 1.05),
    3: (-1.10, 1.05),
    4: (1.10, 1.05),
}

# 미션 상태 → 세그먼트 키 매핑
STATE_TO_SEG = {
    "INIT":             "takeoff",
    "TAKEOFF":          "takeoff",
    "HOME_INIT":        "takeoff",
    "GRID_SEARCH":      "search",
    "MARKER_APPROACH":  "approach",
    "ANTI_SWAY":        "hover",
    "HOVER_CONFIRM":    "hover",
    "MARKER_SAVE":      "hover",
    "RESCUE_ROUTE_PLAN":"rescue",
    "RESCUE_VISIT":     "rescue",
    "RETURN_HOME":      "return",
    "VERTIPORT_ACQUIRE":"land",
    "VISION_SERVO_LAND":"land",
    "LANDED":           "land",
    "ABORT":            "land",
    "EMERGENCY_RETURN": "return",
}


def load_from_bag(bag_path: str):
    """
    Returns:
        segments  dict[str → list[(gx,gy)]]
        events    list[(t_sec, gx, gy, label)]
        states_ts list[(t_sec, state_str)]      — all state transitions
        rescue_order list[int]
    """
    store = get_typestore(Stores.ROS2_JAZZY)

    pose_msgs  = []   # [(t_ns, gx, gy)]
    state_msgs = []   # [(t_ns, state_str)]

    with Reader(bag_path) as reader:
        connections_by_topic = {c.topic: c for c in reader.connections}

        for conn, t_ns, rawdata in reader.messages():
            topic = conn.topic
            if topic == "/localization/pose_grid":
                msg = store.deserialize_cdr(rawdata, conn.msgtype)
                pose_msgs.append((t_ns, msg.pose.position.x, msg.pose.position.y))
            elif topic == "/mission/state":
                msg = store.deserialize_cdr(rawdata, conn.msgtype)
                state_msgs.append((t_ns, msg.data))

    if not pose_msgs or not state_msgs:
        sys.exit("bag is missing required topics")

    pose_msgs.sort(key=lambda x: x[0])
    state_msgs.sort(key=lambda x: x[0])

    t0_ns = min(pose_msgs[0][0], state_msgs[0][0])

    # state timeline: [(t_sec, state)]
    states_ts = [(( t - t0_ns) * 1e-9, s) for t, s in state_msgs]

    # deduplicate consecutive same states
    deduped = []
    for ts_entry in states_ts:
        if not deduped or deduped[-1][1] != ts_entry[1]:
            deduped.append(ts_entry)
    states_ts = deduped

    # ── 포즈를 상태 구간으로 분류 ────────────────────────────────────────────
    # 각 포즈 타임스탬프에 대응하는 상태 찾기 (lower-bound)
    state_t_arr  = np.array([s[0] for s in states_ts])
    state_s_arr  = [s[1] for s in states_ts]

    segments: dict[str, list] = {
        "takeoff": [], "search": [], "approach": [],
        "hover": [], "rescue": [], "return": [], "land": []
    }

    for t_ns, gx, gy in pose_msgs:
        t_sec = (t_ns - t0_ns) * 1e-9
        idx = np.searchsorted(state_t_arr, t_sec, side="right") - 1
        idx = max(0, min(idx, len(state_s_arr) - 1))
        state = state_s_arr[idx]
        seg = STATE_TO_SEG.get(state, "search")
        segments[seg].append((float(gx), float(gy)))

    # ── 이벤트 추출 (상태 전환 시점의 위치) ──────────────────────────────────
    def pose_at(t_sec):
        ts_arr = np.array([p[0] for p in pose_msgs])
        ts_arr_sec = (ts_arr - t0_ns) * 1e-9
        idx = np.searchsorted(ts_arr_sec, t_sec)
        idx = max(0, min(idx, len(pose_msgs) - 1))
        return pose_msgs[idx][1], pose_msgs[idx][2]

    events = []
    rescue_order = []

    # 구조 방문 순서: RESCUE_VISIT 진입 횟수 = 마커 수
    rescue_visits = []
    prev_state = None
    for t_sec, state in states_ts:
        if state == "RESCUE_VISIT" and prev_state != "RESCUE_VISIT":
            gx_ev, gy_ev = pose_at(t_sec)
            rescue_visits.append((t_sec, gx_ev, gy_ev))
        prev_state = state

    # 구조 방문 위치와 마커를 매칭해 rescue_order 복원
    for rv_t, rv_gx, rv_gy in rescue_visits:
        best_mid, best_d = None, 1e9
        for mid, mp in MARKERS.items():
            d = math.sqrt((rv_gx - mp["gx"])**2 + (rv_gy - mp["gy"])**2)
            if d < best_d:
                best_d, best_mid = d, mid
        if best_mid is not None and best_mid not in rescue_order:
            rescue_order.append(best_mid)
            events.append((rv_t, rv_gx, rv_gy, f"구조 #{best_mid}"))

    # 마커 감지 이벤트: APPROACH 진입 시점
    for t_sec, state in states_ts:
        if state == "MARKER_APPROACH":
            gx_ev, gy_ev = pose_at(t_sec)
            events.append((t_sec, gx_ev, gy_ev, "감지"))

    events.sort(key=lambda e: e[0])

    # rescue_order가 비었으면 내림차순 기본값
    if not rescue_order:
        rescue_order = sorted(MARKERS.keys(), reverse=True)

    # 총 임무 시간 계산
    t_end = (pose_msgs[-1][0] - t0_ns) * 1e-9

    return segments, events, states_ts, rescue_order, t_end


def smooth_seg(pts, sigma=3.0, upsample=4):
    if len(pts) < 4:
        return pts
    arr = np.array(pts)
    xs = gaussian_filter1d(arr[:, 0], sigma=sigma)
    ys = gaussian_filter1d(arr[:, 1], sigma=sigma)
    try:
        tck, u = splprep([xs, ys], s=0, k=3)
        u_new = np.linspace(0, 1, len(pts) * upsample)
        xs2, ys2 = splev(u_new, tck)
        return list(zip(xs2, ys2))
    except Exception:
        return list(zip(xs, ys))


def build_serpentine():
    pts = []
    for row in range(GRID_ROWS):
        y = -row * GRID_SPACING
        cols = range(GRID_COLS) if row % 2 == 0 else range(GRID_COLS - 1, -1, -1)
        for col in cols:
            pts.append((GRID_COL_OFFSET + col * GRID_SPACING, y))
    return pts


SERPENTINE = build_serpentine()


def plot_figure(segments, events, states_ts, rescue_order, t_end, out_path):
    # ── 스무딩 ────────────────────────────────────────────────────────────────
    seg_sigma = {
        "search": 5.0, "approach": 3.0, "hover": 2.0,
        "rescue": 4.0, "return": 4.0, "land": 2.0,
    }
    for key, sigma in seg_sigma.items():
        if key in segments and len(segments[key]) >= 4:
            segments[key] = smooth_seg(segments[key], sigma=sigma, upsample=5)

    # ── rcParams ─────────────────────────────────────────────────────────────
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

    C = {
        "search":    "#2166ac",
        "approach":  "#f4a582",
        "hover":     "#d6604d",
        "rescue":    "#762a83",
        "return":    "#1b7837",
        "land":      "#74c476",
        "marker":    "#e08214",
        "detect":    "#b35806",
        "vertiport": "#252525",
        "safe_fill": "#f7f7f7",
        "safe_edge": "#bbbbbb",
        "miss_fill": "#e8f0f8",
        "miss_edge": "#5b8db8",
        "grid_line": "#c8d8e8",
        "wp_dot":    "#aec7e0",
    }

    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # 영역 배경
    ax.add_patch(mpatches.Rectangle((-2, -15), 32, 19,
        linewidth=0.8, edgecolor=C["safe_edge"], facecolor=C["safe_fill"], zorder=0))
    ax.add_patch(mpatches.Rectangle((2, -15), 24, 15,
        linewidth=1.0, edgecolor=C["miss_edge"], facecolor=C["miss_fill"],
        linestyle="-", zorder=1))

    # 그리드 라인
    for row in range(GRID_ROWS + 1):
        y = -row * GRID_SPACING
        ax.plot([2, 28], [y, y], color=C["grid_line"], lw=0.5, zorder=2)
    for col in range(GRID_COLS + 1):
        x = GRID_COL_OFFSET + col * GRID_SPACING
        ax.plot([x, x], [-15, 0], color=C["grid_line"], lw=0.5, zorder=2)

    # 웨이포인트 교점
    for wx, wy in SERPENTINE:
        ax.plot(wx, wy, ".", color=C["wp_dot"], ms=3.5, zorder=3, mew=0)

    # 경로 세그먼트
    def plot_seg(key, color, lw, ls="-", alpha=1.0, z=5):
        pts = segments.get(key, [])
        if len(pts) < 2:
            return
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=color, lw=lw, ls=ls, alpha=alpha, zorder=z,
                solid_capstyle="round", solid_joinstyle="round")

    plot_seg("search",   C["search"],   1.5, "-",  1.0, 5)
    plot_seg("approach", C["approach"], 1.4, "-",  0.9, 6)
    plot_seg("hover",    C["hover"],    1.4, "-",  1.0, 6)
    plot_seg("rescue",   C["rescue"],   1.8, "-",  1.0, 7)
    plot_seg("return",   C["return"],   1.5, "--", 1.0, 7)
    plot_seg("land",     C["land"],     1.2, ":",  0.9, 6)

    # 방향 화살표
    def add_arrows(pts_list, color, n=10, ms=7):
        if not pts_list or len(pts_list) < 4:
            return
        step = max(1, len(pts_list) // n)
        for i in range(step // 2, len(pts_list) - step, step):
            x0, y0 = pts_list[i]
            x1, y1 = pts_list[i + step]
            dx, dy = x1 - x0, y1 - y0
            if dx*dx + dy*dy < 0.04:
                continue
            ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                        arrowprops=dict(arrowstyle="-|>", color=color,
                                        lw=0.7, mutation_scale=ms),
                        zorder=8)

    add_arrows(segments.get("search", []),  C["search"],  n=14, ms=6)
    add_arrows(segments.get("rescue", []),  C["rescue"],  n=6,  ms=7)
    add_arrows(segments.get("return", []),  C["return"],  n=3,  ms=7)

    # ArUco 마커 위치
    for mid, mpos in MARKERS.items():
        mx, my = mpos["gx"], mpos["gy"]
        ax.scatter([mx], [my], marker="s", s=140, facecolor="white",
                   edgecolor="#7a3f00", linewidth=1.0, zorder=12)
        ax.scatter([mx], [my], marker="s", s=78, facecolor=C["marker"],
                   edgecolor="white", linewidth=0.8, zorder=13)

        dx, dy = MARKER_LABEL_OFFSETS[mid]
        ax.annotate(
            f"M{mid}",
            xy=(mx, my),
            xytext=(mx + dx, my + dy),
            ha="center",
            va="center",
            fontsize=8,
            color="#7a3f00",
            fontweight="bold",
            zorder=15,
            bbox=dict(
                boxstyle="round,pad=0.22",
                facecolor="white",
                edgecolor=C["marker"],
                linewidth=0.7,
                alpha=0.96,
            ),
            arrowprops=dict(
                arrowstyle="-",
                color=C["marker"],
                lw=0.7,
                shrinkA=4,
                shrinkB=6,
                alpha=0.9,
            ),
        )

    # 감지 이벤트 마커
    for t_ev, gx_ev, gy_ev, label in events:
        if "감지" in label:
            ax.plot(gx_ev, gy_ev, "^", color=C["detect"],
                    ms=6, mec="white", mew=0.6, zorder=9)

    # 구조 방문 다이아몬드
    rescue_seq = {mid: i + 1 for i, mid in enumerate(rescue_order)}
    for t_ev, gx_ev, gy_ev, label in events:
        if "구조" in label:
            mid = int(label.split("#")[1])
            seq = rescue_seq.get(mid, "?")
            ax.plot(gx_ev, gy_ev, "D", color=C["rescue"],
                    ms=7, mec="white", mew=0.8, zorder=11, alpha=0.85)
            ax.text(gx_ev, gy_ev, str(seq), color="white",
                    fontsize=6.5, ha="center", va="center",
                    fontweight="bold", zorder=12)

    # 버티포트
    ax.add_patch(plt.Circle((0, 0), 1.5, color=C["vertiport"], fill=False,
                             lw=1.0, ls="-", zorder=4, alpha=0.4))
    ax.plot(0, 0, "o", color=C["vertiport"], ms=9,
            mec=C["vertiport"], mew=1.0, zorder=12)
    ax.plot(0, 0, "+", color="white", ms=7, mew=1.2, zorder=13)
    ax.text(1.8, 0.3, "Vertiport", color=C["vertiport"],
            fontsize=7.5, va="center", style="italic")

    # 영역 레이블
    ax.text(14, -16.0, "Mission Area  (24 m × 15 m)",
            color=C["miss_edge"], ha="center", va="top", fontsize=7.5, style="italic")
    ax.text(-1.5, -7.5, "Safe\nArea", color=C["safe_edge"],
            ha="center", va="center", fontsize=7, style="italic", rotation=90)

    # 축
    ax.set_xlim(-3, 30)
    ax.set_ylim(-17, 4.5)
    ax.set_aspect("equal")
    ax.set_xlabel("$x$ (m, east)", fontsize=9)
    ax.set_ylabel("$y$ (m, north)", fontsize=9)
    ax.tick_params(labelsize=8)
    ax.set_xticks(range(0, 30, 3))
    ax.set_yticks(range(-15, 4, 3))
    ax.xaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(1.5))
    ax.yaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(1.5))
    ax.set_xticklabels([f"{v}" for v in range(0, 30, 3)], fontsize=7.5)
    ax.set_yticklabels([f"{v}" for v in range(-15, 4, 3)], fontsize=7.5)

    # 범례
    legend_elems = [
        Line2D([0],[0], color=C["search"],   lw=1.5,           label="Grid search"),
        Line2D([0],[0], color=C["approach"], lw=1.4,           label="Marker approach"),
        Line2D([0],[0], color=C["hover"],    lw=1.4,           label="Hover / confirm"),
        Line2D([0],[0], color=C["rescue"],   lw=1.8,           label="Rescue visit"),
        Line2D([0],[0], color=C["return"],   lw=1.5, ls="--",  label="Return home"),
        Line2D([0],[0], color=C["land"],     lw=1.2, ls=":",   label="Landing approach"),
        Line2D([0],[0], marker="s", color="none", mfc=C["marker"],
               ms=7, mew=0, label="ArUco marker"),
        Line2D([0],[0], marker="^", color="none", mfc=C["detect"],
               ms=6, mew=0, label="Detection point"),
        Line2D([0],[0], marker="D", color="none", mfc=C["rescue"],
               ms=7, mew=0, label="Rescue waypoint (order)"),
        Line2D([0],[0], marker="o", color="none", mfc=C["vertiport"],
               ms=7, mew=0, label="Vertiport"),
    ]
    ax.legend(handles=legend_elems, loc="center left",
              bbox_to_anchor=(1.01, 0.5),
              fontsize=7.2, ncol=1, handlelength=2.2, handleheight=1.0,
              borderpad=0.7, labelspacing=0.45, handletextpad=0.5,
              framealpha=0.92, edgecolor="0.6")

    # 제목
    ax.set_title(
        "Mission Flight Path: Grid Search, ArUco Detection, and Rescue Visit Sequence",
        fontsize=9.5, pad=6
    )

    # rescue order text
    ro_str = "→".join(str(m) for m in rescue_order) if rescue_order else "N/A"
    n_detected = sum(1 for _, _, _, lbl in events if "감지" in lbl)
    t_str = f"{int(t_end)} s" if t_end < 3600 else "—"

    plt.tight_layout(rect=[0, 0, 0.82, 1])
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=200, bbox_inches="tight", facecolor="white")
    print(f"저장: {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory (contains metadata.yaml)")
    parser.add_argument("--out", default="/home/dev/drone_middle/reports/mission_path_real.png")
    args = parser.parse_args()

    print(f"bag: {args.bag}")
    segments, events, states_ts, rescue_order, t_end = load_from_bag(args.bag)

    # 통계 출력
    for seg, pts in segments.items():
        if pts:
            print(f"  {seg:10s}: {len(pts):5d} points")

    print(f"  events     : {len(events)}")
    print(f"  rescue_order: {rescue_order}")
    print(f"  duration   : {t_end:.1f} s")

    for t_sec, state in states_ts:
        print(f"  {t_sec:6.1f}s  {state}")

    plot_figure(segments, events, states_ts, rescue_order, t_end, args.out)


if __name__ == "__main__":
    main()
