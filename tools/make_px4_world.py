#!/usr/bin/env python3
"""Generate a PX4 SITL world from the standalone sprint-grid world.

The checked-in `sprint_grid_world.sdf` contains a fixed camera rig for fast
perception testing without a vehicle. PX4 SITL should use the vehicle-mounted
camera instead, so this helper strips that fixed rig and optionally renames the
world for PX4's `PX4_GZ_WORLD` lookup.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


DEFAULT_INPUT = Path("src/sprint_drone/worlds/sprint_grid_world.sdf")
DEFAULT_OUTPUT = Path("external/px4_worlds/sprint_grid_world_px4.sdf")


def remove_model(world: ET.Element, model_name: str) -> bool:
    for model in list(world.findall("model")):
        if model.get("name") == model_name:
            world.remove(model)
            return True
    return False


def generate_px4_world(input_path: Path, output_path: Path, world_name: str) -> bool:
    tree = ET.parse(input_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"No <world> element found in {input_path}")

    world.set("name", world_name)
    removed_camera_rig = remove_model(world, "sprint_camera_rig")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return removed_camera_rig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"source SDF world (default: {DEFAULT_INPUT})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"generated PX4 SDF world (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--world-name",
        default="sprint_grid_world_px4",
        help="world name written into the SDF and used by PX4_GZ_WORLD",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    removed = generate_px4_world(args.input, args.output, args.world_name)
    status = "removed sprint_camera_rig" if removed else "sprint_camera_rig not present"
    print(f"Wrote {args.output} ({status}, world={args.world_name})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
