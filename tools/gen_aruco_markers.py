#!/usr/bin/env python3
"""Generate ArUco marker PNG images for Gazebo Harmonic textures.

Requires: opencv-python (pip3 install opencv-python)

Usage:
    python3 tools/gen_aruco_markers.py
    python3 tools/gen_aruco_markers.py --ids 1 2 3 4 --size 512 --output src/sprint_drone/models/aruco_textures/materials/textures/
"""
import argparse
import sys
from pathlib import Path


def generate(ids: list[int], size_px: int, output_dir: Path) -> None:
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("opencv-python not installed. Run: pip3 install opencv-python")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    output_dir.mkdir(parents=True, exist_ok=True)
    for mid in ids:
        if mid < 0 or mid >= 50:
            print(f"WARNING: ID {mid} out of range for DICT_4X4_50 (0–49), skipping")
            continue
        img = cv2.aruco.generateImageMarker(dictionary, mid, size_px)
        # Add white border (10% of size) so the black marker border is clearly visible
        border = size_px // 10
        padded = cv2.copyMakeBorder(img, border, border, border, border,
                                    cv2.BORDER_CONSTANT, value=255)
        path = output_dir / f"aruco_{mid}.png"
        cv2.imwrite(str(path), padded)
        print(f"  aruco_{mid}.png  ({padded.shape[1]}×{padded.shape[0]} px)  → {path}")

    print(f"\n{len(ids)} marker(s) written to {output_dir}")


def main() -> None:
    default_out = Path(__file__).parent.parent / \
        "src/sprint_drone/models/aruco_textures/materials/textures"

    parser = argparse.ArgumentParser(
        description="Generate ArUco DICT_4X4_50 marker PNG files for Gazebo textures")
    parser.add_argument("--ids", nargs="+", type=int, default=[1, 2, 3, 4],
                        help="Marker IDs to generate (default: 1 2 3 4)")
    parser.add_argument("--size", type=int, default=512,
                        help="Marker image size in pixels before border (default: 512)")
    parser.add_argument("--output", type=Path, default=default_out,
                        help=f"Output directory (default: {default_out})")
    args = parser.parse_args()

    print(f"Generating ArUco DICT_4X4_50 markers: IDs {args.ids}")
    generate(args.ids, args.size, args.output)


if __name__ == "__main__":
    main()
