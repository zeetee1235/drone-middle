#!/usr/bin/env python3
"""Randomly place 4 ArUco markers at grid intersections and write a Gazebo world file.

Usage:
    python3 tools/spawn_markers.py                        # random seed, preview only
    python3 tools/spawn_markers.py --seed 7               # deterministic run
    python3 tools/spawn_markers.py --seed 7 --write       # overwrite sprint_grid_world.sdf

The script reads sprint_grid_world.sdf as a template and replaces the
<!-- MARKER_MODELS --> placeholder block with the generated marker models.
"""
import argparse
import random
import re
from pathlib import Path
from typing import List, Tuple

# ── Grid geometry ────────────────────────────────────────────────────────────
# Mission area origin (4, 4), 3m spacing, 9 cols × 6 rows = 54 intersections
GRID_ORIGIN_X: float = 4.0
GRID_ORIGIN_Y: float = 4.0
GRID_SPACING: float = 3.0
GRID_COLS: int = 9   # x = 4, 7, 10, 13, 16, 19, 22, 25, 28
GRID_ROWS: int = 6   # y = 4, 7, 10, 13, 16, 19

MARKER_IDS: List[int] = [1, 2, 3, 4]
MARKER_Z: float = 0.043          # just above grid lines (z=0.03)
MARKER_SIZE: float = 0.5         # 50 cm per contest rules

WORLD_TEMPLATE = Path(__file__).parent.parent / \
    "src/sprint_drone/worlds/sprint_grid_world.sdf"

MARKER_BLOCK_START = "<!-- MARKER_MODELS -->"
MARKER_BLOCK_END = "<!-- /MARKER_MODELS -->"


def all_intersections() -> List[Tuple[float, float]]:
    return [
        (GRID_ORIGIN_X + col * GRID_SPACING,
         GRID_ORIGIN_Y + row * GRID_SPACING)
        for col in range(GRID_COLS)
        for row in range(GRID_ROWS)
    ]


def pick_placements(seed: int) -> List[Tuple[int, float, float]]:
    rng = random.Random(seed)
    chosen = rng.sample(all_intersections(), len(MARKER_IDS))
    return [(mid, x, y) for mid, (x, y) in zip(MARKER_IDS, chosen)]


def sdf_marker(mid: int, x: float, y: float) -> str:
    return f"""\
    <model name="aruco_marker_{mid}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {MARKER_Z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{MARKER_SIZE} {MARKER_SIZE} 0.005</size></box>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <ambient>1 1 1 1</ambient>
            <pbr>
              <metal>
                <albedo_map>model://aruco_textures/materials/textures/aruco_{mid}.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>"""


def build_fragment(placements: List[Tuple[int, float, float]], seed: int) -> str:
    lines = [f"    <!-- spawn_markers.py seed={seed} -->"]
    for mid, x, y in placements:
        lines.append(sdf_marker(mid, x, y))
    return "\n".join(lines)


def apply_to_world(fragment: str) -> str:
    template = WORLD_TEMPLATE.read_text()
    managed_block = f"{MARKER_BLOCK_START}\n{fragment}\n{MARKER_BLOCK_END}"
    block_re = re.compile(
        rf"{re.escape(MARKER_BLOCK_START)}.*?{re.escape(MARKER_BLOCK_END)}",
        re.DOTALL,
    )
    if block_re.search(template):
        return block_re.sub(managed_block, template, count=1)

    if MARKER_BLOCK_START not in template:
        raise ValueError(
            f"Placeholder '{MARKER_BLOCK_START}' not found in {WORLD_TEMPLATE}. "
            "Cannot inject marker models.")
    return template.replace(MARKER_BLOCK_START, managed_block, 1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Randomly place 4 ArUco markers at grid intersections")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed (default: random)")
    parser.add_argument("--positions", type=str, default=None,
                        help="Fixed placements: 'x1,y1 x2,y2 x3,y3 x4,y4' (Gazebo world coords)")
    parser.add_argument("--write", action="store_true",
                        help=f"Overwrite {WORLD_TEMPLATE} with new marker placements")
    parser.add_argument("--list-all", action="store_true",
                        help="Print all 54 valid intersection coordinates and exit")
    args = parser.parse_args()

    if args.list_all:
        pts = all_intersections()
        print(f"All {len(pts)} grid intersections:")
        for i, (x, y) in enumerate(pts):
            print(f"  [{i:2d}]  x={x:.0f}  y={y:.0f}")
        return

    if args.positions:
        raw = args.positions.strip().split()
        if len(raw) != 4:
            parser.error("--positions requires exactly 4 'x,y' pairs")
        placements = []
        valid = set(all_intersections())
        for mid, pair in zip(MARKER_IDS, raw):
            x, y = (float(v) for v in pair.split(","))
            if (x, y) not in valid:
                print(f"  WARNING: ({x:.0f},{y:.0f}) is not a grid intersection")
            placements.append((mid, x, y))
        seed = 0
    else:
        seed = args.seed if args.seed is not None else random.randrange(10000)
        placements = pick_placements(seed)

    print(f"Marker placements (seed={seed}):")
    for mid, x, y in placements:
        print(f"  ID {mid}  →  ({x:.0f}, {y:.0f})")

    fragment = build_fragment(placements, seed)

    if args.write:
        world_text = apply_to_world(fragment)
        WORLD_TEMPLATE.write_text(world_text)
        print(f"\nWorld file updated: {WORLD_TEMPLATE}")
    else:
        print(f"\n--- SDF fragment (use --write to apply) ---")
        print(fragment)


if __name__ == "__main__":
    main()
