#!/usr/bin/env python3
"""
test_full_mission_log.py — 전체 임무 시뮬레이션 (ROS2/Gazebo 없이)

mission_core.cpp 상태 기계를 Python으로 재현하고
서펜타인 그리드 탐색 → ArUco 마커 4개 인식 → 역순 구조방문 → 복귀/착륙까지
드론 좌표 + 상태 전이를 타임라인으로 출력.

프레임 규약:
  grid frame  : origin = vertiport, x=east, y=north(양수)/south(음수)
  Gazebo world: x=east, y=north  →  gz = grid + (START_X, START_Y)
"""

import math

# ── 세계 상수 ──────────────────────────────────────────────────────────────────
START_X, START_Y = 2.0, 19.0   # vertiport Gazebo 좌표
CRUISE_Z         = 2.0          # 순항 고도 (m)

GRID_SPACING     = 3.0
GRID_COL_OFFSET  = 2.0
GRID_ROWS        = 6
GRID_COLS        = 9

V_MAX            = 3.0          # m/s
WAYPOINT_TOL     = 1.5          # 웨이포인트 진입 반경 (m)
MARKER_DETECT_R  = 1.5          # 마커 인식 반경 (m)
HOVER_DUR        = 3.0          # 호버 확인 시간 (s)
DT               = 0.1          # 시뮬 스텝 (s)

# ── 마커 배치 (world SDF, grid frame) ─────────────────────────────────────────
# Gazebo (19,19)→grid (17,0)   /  (4,16)→grid (2,-3)
# Gazebo (10,4)→grid (8,-15)   /  (25,4)→grid (23,-15)
MARKERS = {
    1: {"gx": 17.0, "gy":  0.0},
    2: {"gx":  2.0, "gy": -3.0},
    3: {"gx":  8.0, "gy":-15.0},
    4: {"gx": 23.0, "gy":-15.0},
}

# ── 서펜타인 경로 생성 ────────────────────────────────────────────────────────
def build_serpentine():
    pts = []
    for row in range(GRID_ROWS):
        y = -row * GRID_SPACING
        cols = range(GRID_COLS) if row % 2 == 0 else range(GRID_COLS-1, -1, -1)
        for col in cols:
            pts.append((GRID_COL_OFFSET + col * GRID_SPACING, y))
    return pts

SERPENTINE = build_serpentine()

# ── 유틸 ──────────────────────────────────────────────────────────────────────
def dist(ax, ay, bx, by):
    return math.sqrt((ax-bx)**2 + (ay-by)**2)

def move_toward(cx, cy, tx, ty, speed, dt):
    d = dist(cx, cy, tx, ty)
    if d < 1e-6:
        return cx, cy
    step = min(speed * dt, d)
    return cx + (tx-cx)/d*step, cy + (ty-cy)/d*step

def gz(gx, gy):
    return gx + START_X, gy + START_Y

# ── 상태 기계 ─────────────────────────────────────────────────────────────────
PHASES = [
    "INIT","TAKEOFF","HOME_INIT","GRID_SEARCH",
    "MARKER_APPROACH","ANTI_SWAY","HOVER_CONFIRM","MARKER_SAVE",
    "RESCUE_ROUTE_PLAN","RESCUE_VISIT",
    "RETURN_HOME","VERTIPORT_ACQUIRE","VISION_SERVO_LAND","LANDED",
]

# ── 로그 버퍼 ─────────────────────────────────────────────────────────────────
events = []   # (t, phase, grid_x, grid_y, gz_x, gz_y, note)

def log_event(t, phase, gx, gy, note=""):
    wx, wy = gz(gx, gy)
    events.append((t, phase, gx, gy, wx, wy, note))

def phase_changed(t, phase, gx, gy, note=""):
    log_event(t, phase, gx, gy, note)
    print(f"  [{t:7.1f}s] ▶ {phase:<22} grid({gx:+6.2f},{gy:+6.2f})  {note}")

# ── 메인 시뮬레이션 ───────────────────────────────────────────────────────────
def run():
    t     = 0.0
    gx    = 0.0   # grid frame 좌표 (vertiport = 0,0)
    gy    = 0.0
    gz_z  = 0.0
    phase = "INIT"

    confirmed  = {}    # {marker_id: {"gx":, "gy":}}
    detected   = set() # 탐색 중 감지된 마커 ID
    visited    = []    # 구조 방문 완료 목록
    rescue_order = []  # 역순 방문 순서

    serpentine_idx = 0
    hover_start    = None
    approach_target = None   # (gx, gy, marker_id)
    rescue_target   = None   # (gx, gy, marker_id)
    rescue_idx      = 0
    home_init_start = None

    LOG_TICK = max(1, int(0.5 / DT))  # 0.5초마다 상세 좌표 로그

    print("=" * 80)
    print("  전체 임무 시뮬레이션")
    marker_summary = {k: "grid({:.0f},{:.0f})".format(v["gx"], v["gy"]) for k, v in MARKERS.items()}
    print(f"  마커 배치: {marker_summary}")
    print(f"  서펜타인 : {len(SERPENTINE)}개 웨이포인트 | v_max={V_MAX}m/s")
    print("=" * 80)
    print()

    tick = 0
    prev_phase = None

    while phase != "LANDED" and t < 600.0:
        t    = tick * DT
        tick += 1

        if phase != prev_phase:
            phase_changed(t, phase, gx, gy)
            prev_phase = phase

        # ── INIT ──────────────────────────────────────────────────────────────
        if phase == "INIT":
            phase = "TAKEOFF"

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        elif phase == "TAKEOFF":
            gz_z = min(gz_z + 1.7 * DT, CRUISE_Z)
            if gz_z >= CRUISE_Z - 0.05:
                phase = "HOME_INIT"
                home_init_start = t

        # ── HOME_INIT ─────────────────────────────────────────────────────────
        elif phase == "HOME_INIT":
            if t - home_init_start >= 1.0:   # 1초 hover로 홈 초기화
                phase = "GRID_SEARCH"
                serpentine_idx = 0

        # ── GRID_SEARCH ───────────────────────────────────────────────────────
        elif phase == "GRID_SEARCH":
            if len(confirmed) >= 4:
                phase = "RESCUE_ROUTE_PLAN"
                continue

            # 웨이포인트로 이동
            if serpentine_idx < len(SERPENTINE):
                tx, ty = SERPENTINE[serpentine_idx]
                gx, gy = move_toward(gx, gy, tx, ty, V_MAX, DT)
                if dist(gx, gy, tx, ty) < WAYPOINT_TOL:
                    serpentine_idx += 1
            else:
                # 서펜타인 끝 — 마커 부족하면 ABORT 대신 HOME
                phase = "RETURN_HOME"
                continue

            # 마커 감지 체크
            for mid, mpos in MARKERS.items():
                if mid in confirmed or mid in detected:
                    continue
                d = dist(gx, gy, mpos["gx"], mpos["gy"])
                if d < MARKER_DETECT_R:
                    detected.add(mid)
                    approach_target = (mpos["gx"], mpos["gy"], mid)
                    phase = "MARKER_APPROACH"
                    log_event(t, "MARKER_DETECT", gx, gy,
                              f"marker_id={mid} d={d:.2f}m grid({mpos['gx']:.0f},{mpos['gy']:.0f})")
                    print(f"  [{t:7.1f}s]   🔍 마커#{mid} 감지!  d={d:.2f}m  "
                          f"grid({mpos['gx']:.0f},{mpos['gy']:.0f})")
                    break

        # ── MARKER_APPROACH ───────────────────────────────────────────────────
        elif phase == "MARKER_APPROACH":
            if approach_target:
                tx, ty, mid = approach_target
                gx, gy = move_toward(gx, gy, tx, ty, V_MAX * 0.5, DT)
                if dist(gx, gy, tx, ty) < 0.3:
                    phase = "ANTI_SWAY"

        # ── ANTI_SWAY ─────────────────────────────────────────────────────────
        elif phase == "ANTI_SWAY":
            # 0.5초 진동 감쇠 후 확인
            if hover_start is None:
                hover_start = t
            if t - hover_start >= 0.5:
                hover_start = None
                phase = "HOVER_CONFIRM"

        # ── HOVER_CONFIRM ─────────────────────────────────────────────────────
        elif phase == "HOVER_CONFIRM":
            if hover_start is None:
                hover_start = t
            if t - hover_start >= HOVER_DUR:
                hover_start = None
                phase = "MARKER_SAVE"

        # ── MARKER_SAVE ───────────────────────────────────────────────────────
        elif phase == "MARKER_SAVE":
            if approach_target:
                mid = approach_target[2]
                confirmed[mid] = {"gx": approach_target[0], "gy": approach_target[1]}
                approach_target = None
                log_event(t, "MARKER_SAVED", gx, gy, f"marker_id={mid}  total={len(confirmed)}/4")
                print(f"  [{t:7.1f}s]   ✅ 마커#{mid} 저장!  확인된 마커: {sorted(confirmed.keys())}")
            phase = "GRID_SEARCH" if len(confirmed) < 4 else "RESCUE_ROUTE_PLAN"

        # ── RESCUE_ROUTE_PLAN ─────────────────────────────────────────────────
        elif phase == "RESCUE_ROUTE_PLAN":
            rescue_order = sorted(confirmed.keys(), reverse=True)  # 역순 ID
            rescue_idx = 0
            print(f"  [{t:7.1f}s]   📋 구조 방문 순서: {rescue_order}")
            phase = "RESCUE_VISIT"

        # ── RESCUE_VISIT ──────────────────────────────────────────────────────
        elif phase == "RESCUE_VISIT":
            if rescue_idx >= len(rescue_order):
                phase = "RETURN_HOME"
                continue

            mid = rescue_order[rescue_idx]
            mpos = confirmed[mid]

            if rescue_target is None or rescue_target[2] != mid:
                rescue_target = (mpos["gx"], mpos["gy"], mid)
                hover_start = None
                print(f"  [{t:7.1f}s]   🚁 마커#{mid} 방문 중  grid({mpos['gx']:.0f},{mpos['gy']:.0f})")

            tx, ty, _ = rescue_target
            d = dist(gx, gy, tx, ty)

            if d > 0.3:
                gx, gy = move_toward(gx, gy, tx, ty, V_MAX, DT)
            else:
                # 목표 도달 → 호버
                if hover_start is None:
                    hover_start = t
                    log_event(t, "RESCUE_HOVER", gx, gy, f"marker_id={mid}")
                    print(f"  [{t:7.1f}s]   🎯 마커#{mid} 도달!  {HOVER_DUR}s 호버 시작")
                if t - hover_start >= HOVER_DUR:
                    visited.append(mid)
                    rescue_idx += 1
                    hover_start = None
                    rescue_target = None
                    print(f"  [{t:7.1f}s]   ✈  마커#{mid} 방문 완료  visited={visited}")
                    if rescue_idx >= len(rescue_order):
                        phase = "RETURN_HOME"

        # ── RETURN_HOME ───────────────────────────────────────────────────────
        elif phase == "RETURN_HOME":
            gx, gy = move_toward(gx, gy, 0.0, 0.0, V_MAX, DT)
            if dist(gx, gy, 0.0, 0.0) < WAYPOINT_TOL:
                phase = "VERTIPORT_ACQUIRE"

        # ── VERTIPORT_ACQUIRE ─────────────────────────────────────────────────
        elif phase == "VERTIPORT_ACQUIRE":
            gx, gy = move_toward(gx, gy, 0.0, 0.0, V_MAX * 0.3, DT)
            if dist(gx, gy, 0.0, 0.0) < 0.3:
                phase = "VISION_SERVO_LAND"
                hover_start = t

        # ── VISION_SERVO_LAND ─────────────────────────────────────────────────
        elif phase == "VISION_SERVO_LAND":
            gz_z = max(gz_z - 0.5 * DT, 0.0)
            if gz_z <= 0.05:
                phase = "LANDED"

    # 최종 상태 출력
    phase_changed(t, phase, gx, gy, "임무 종료")
    print()

    # ── 이벤트 타임라인 요약 ──────────────────────────────────────────────────
    print("=" * 80)
    print("  이벤트 타임라인 요약")
    print("=" * 80)
    hdr = f"{'time':>8}  {'phase':<22} {'grid_x':>8} {'grid_y':>8} {'gz_x':>7} {'gz_y':>7}  note"
    print(hdr)
    print("-" * len(hdr))
    for ev in events:
        t_, ph, gx_, gy_, wx_, wy_, note_ = ev
        print(f"{t_:>8.1f}  {ph:<22} {gx_:>+8.3f} {gy_:>+8.3f} {wx_:>7.3f} {wy_:>7.3f}  {note_}")

    # ── 서펜타인 진행 통계 ────────────────────────────────────────────────────
    print()
    print("=" * 80)
    print("  서펜타인 웨이포인트 상세")
    print("=" * 80)
    print(f"  총 {len(SERPENTINE)}개  /  탐색 완료: {min(serpentine_idx, len(SERPENTINE))}개")
    print()
    print(f"  {'idx':>4}  {'grid_x':>7} {'grid_y':>7}  {'gz_x':>7} {'gz_y':>7}")
    print(f"  {'----':>4}  {'------':>7} {'------':>7}  {'-----':>7} {'-----':>7}")
    for i, (wx, wy) in enumerate(SERPENTINE):
        mark = "  ← 마커 근방" if any(
            dist(wx, wy, MARKERS[m]["gx"], MARKERS[m]["gy"]) < GRID_SPACING
            for m in MARKERS) else ""
        marker_ids = [m for m in MARKERS
                      if dist(wx, wy, MARKERS[m]["gx"], MARKERS[m]["gy"]) < GRID_SPACING]
        if marker_ids:
            mark = f"  ← 마커 #{marker_ids[0]} 근방"
        reached = "✓" if i < serpentine_idx else " "
        print(f"  {i:>4} {reached} {wx:>7.1f} {wy:>7.1f}  {wx+START_X:>7.1f} {wy+START_Y:>7.1f}{mark}")

    # ── 구조 방문 결과 ────────────────────────────────────────────────────────
    print()
    print("=" * 80)
    print("  구조 방문 결과")
    print("=" * 80)
    print(f"  발견된 마커 (인식 순): {sorted(confirmed.keys())}")
    print(f"  방문 순서 (역순 ID):   {rescue_order}")
    print(f"  방문 완료:             {visited}")
    print(f"  총 임무 시간:          {t:.1f}s ({t/60:.1f}분)")
    print()

    # ── 최종 검증 ─────────────────────────────────────────────────────────────
    print("=" * 80)
    print("  최종 검증")
    print("=" * 80)
    checks = [
        ("마커 4개 모두 발견", len(confirmed) == 4),
        ("구조 방문 역순(4→3→2→1)", rescue_order == [4, 3, 2, 1]),
        ("구조 방문 4개 완료", len(visited) == 4),
        ("방문 완료 순서 일치", visited == rescue_order),
        ("최종 상태 LANDED", phase == "LANDED"),
        ("복귀 위치 vertiport(grid≈0,0)", dist(gx, gy, 0.0, 0.0) < 0.5),
    ]
    all_ok = True
    for desc, ok in checks:
        sym = "✓" if ok else "✗"
        print(f"  [{sym}] {desc}")
        if not ok:
            all_ok = False
    print()
    if all_ok:
        print("  ✅ 전체 임무 검증 통과")
    else:
        print("  ❌ 일부 검증 실패")
    print()


if __name__ == "__main__":
    run()
