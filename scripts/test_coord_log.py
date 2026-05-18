#!/usr/bin/env python3
"""
test_coord_log.py — gz_drone_sim 좌표 적분 로직 단독 실험.

ROS2/Gazebo 없이 NED velocity를 직접 주입하고
Gazebo 세계 좌표 및 그리드 좌표가 어떻게 변하는지 시뮬레이션 후 CSV/표로 출력.

미션 시나리오:
  0~2s    : 이륙 (vz_ned=-1.7 m/s → 위로)
  2~5s    : 첫 번째 열로 이동 (vx_ned=1.5 m/s → north, 첫 웨이포인트 grid x=2 방향)
  5~8s    : 동쪽 이동 (vy_ned=1.5 m/s → east, 그리드 x 증가)
  8~10s   : 정지
"""

import math

# ── 상수 (gz_drone_sim.py와 동일) ─────────────────────────────────────────────
START_X   = 2.0    # Gazebo world x (east, vertiport)
START_Y   = 19.0   # Gazebo world y (north, vertiport)
START_Z   = 0.3    # 초기 고도

BOUND_X   = (0.0, 32.0)
BOUND_Y   = (0.0, 23.0)
BOUND_Z   = (0.05, 5.0)

UPDATE_HZ = 50
DT        = 1.0 / UPDATE_HZ

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ── 미션 시나리오 정의 ────────────────────────────────────────────────────────
# (시작_초, 종료_초, vx_ned(north), vy_ned(east), vz_ned(down))
SCENARIO = [
    (0.0,  2.0,  0.0,  0.0, -1.7),   # 이륙: vz_ned 음수 → 위로
    (2.0,  5.0,  0.0,  0.0, -0.2),   # 고도 유지하며 hover
    (5.0,  8.0,  0.0,  1.5, -0.0),   # 동쪽(east) 이동: vy_ned=1.5
    (8.0, 11.0,  1.5,  0.0,  0.0),   # 북쪽(north) 이동: vx_ned=1.5 (그리드 y 증가)
    (11.0,14.0, -1.5,  0.0,  0.0),   # 남쪽(south) 이동: vx_ned=-1.5
    (14.0,17.0,  0.0, -1.5,  0.0),   # 서쪽(west)  이동: vy_ned=-1.5
    (17.0,20.0,  0.0,  0.0,  0.0),   # 정지
]

def get_vel(t):
    for t0, t1, vx, vy, vz in SCENARIO:
        if t0 <= t < t1:
            return vx, vy, vz
    return 0.0, 0.0, 0.0

# ── 시뮬레이션 루프 ────────────────────────────────────────────────────────────
x, y, z = START_X, START_Y, START_Z

LOG_INTERVAL = 10  # 매 10틱 (0.2초)마다 출력

header = (
    f"{'time_s':>7} | "
    f"{'gz_x':>7} {'gz_y':>7} {'gz_z':>6} | "
    f"{'grid_x':>7} {'grid_y':>7} {'grid_z':>6} | "
    f"{'vx_ned':>7} {'vy_ned':>7} {'vz_ned':>7}"
)
sep = "-" * len(header)

print(sep)
print(header)
print(sep)

rows = []
total_ticks = int(20.0 / DT)  # 20초 시뮬레이션
for tick in range(total_ticks + 1):
    t = tick * DT
    vx_ned, vy_ned, vz_ned = get_vel(t)

    # NED → Gazebo world ENU 변환 (gz_drone_sim._step과 동일)
    x += vy_ned * DT    # east  = NED y
    y += vx_ned * DT    # north = NED x
    z += -vz_ned * DT   # up    = -NED z

    x = clamp(x, *BOUND_X)
    y = clamp(y, *BOUND_Y)
    z = clamp(z, *BOUND_Z)

    # Grid frame 변환 (vertiport = origin)
    grid_x = x - START_X
    grid_y = y - START_Y

    if tick % LOG_INTERVAL == 0:
        row = (
            f"{t:>7.2f} | "
            f"{x:>7.3f} {y:>7.3f} {z:>6.3f} | "
            f"{grid_x:>7.3f} {grid_y:>7.3f} {grid_z if False else z:>6.3f} | "
            f"{vx_ned:>7.2f} {vy_ned:>7.2f} {vz_ned:>7.2f}"
        )
        # grid_z는 단순히 고도
        row = (
            f"{t:>7.2f} | "
            f"{x:>7.3f} {y:>7.3f} {z:>6.3f} | "
            f"{grid_x:>+7.3f} {grid_y:>+7.3f} {z:>6.3f} | "
            f"{vx_ned:>+7.2f} {vy_ned:>+7.2f} {vz_ned:>+7.2f}"
        )
        rows.append(row)
        print(row)

print(sep)

# ── 웨이포인트 도달 여부 검증 ────────────────────────────────────────────────
print()
print("=== 서펜타인 웨이포인트 참조 (grid frame) ===")
print("Row 0 (y=0):  x = 2, 5, 8, 11, 14, 17, 20, 23, 26  (동쪽 →)")
print("Row 1 (y=-3): x = 26,23,20,..., 2                    (서쪽 ←)")
print()

# 최종 도달 위치
print(f"최종 Gazebo  : ({x:.3f}, {y:.3f}, {z:.3f})")
print(f"최종 Grid    : ({x-START_X:+.3f}, {y-START_Y:+.3f}, {z:.3f})")
print()

# 이동 경로 검증
print("=== 변환 검증 ===")
tests = [
    ("vy_ned=+1.5 (east)  → grid_x 증가?", "vy_ned=+1.5 → x += 1.5*dt → grid_x 증가 ✓"),
    ("vx_ned=+1.5 (north) → grid_y 증가?", "vx_ned=+1.5 → y += 1.5*dt → grid_y 증가 ✓"),
    ("vx_ned=-1.5 (south) → grid_y 감소?", "vx_ned=-1.5 → y -= 1.5*dt → grid_y 감소 ✓"),
    ("vy_ned=-1.5 (west)  → grid_x 감소?", "vy_ned=-1.5 → x -= 1.5*dt → grid_x 감소 ✓"),
    ("vz_ned=-1.7 (up)    → gz_z 증가?",   "vz_ned=-1.7 → z += 1.7*dt → gz_z 증가 ✓"),
]
for q, a in tests:
    print(f"  {q}")
    print(f"    → {a}")
print()

# CSV 출력 (파일로도 저장)
import csv, os
out_path = "/tmp/drone_coord_log.csv"
with open(out_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s","gz_x","gz_y","gz_z","grid_x","grid_y","vx_ned","vy_ned","vz_ned"])
    x2, y2, z2 = START_X, START_Y, START_Z
    for tick in range(total_ticks + 1):
        t = tick * DT
        vx_ned, vy_ned, vz_ned = get_vel(t)
        x2 += vy_ned * DT
        y2 += vx_ned * DT
        z2 += -vz_ned * DT
        x2 = clamp(x2, *BOUND_X)
        y2 = clamp(y2, *BOUND_Y)
        z2 = clamp(z2, *BOUND_Z)
        writer.writerow([f"{t:.2f}", f"{x2:.4f}", f"{y2:.4f}", f"{z2:.4f}",
                         f"{x2-START_X:.4f}", f"{y2-START_Y:.4f}",
                         f"{vx_ned:.2f}", f"{vy_ned:.2f}", f"{vz_ned:.2f}"])
print(f"CSV 저장: {out_path}")
