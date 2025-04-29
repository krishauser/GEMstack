#!/usr/bin/env python3
"""
Convert an overhead screenshot of drivable (black) vs non‑drivable (white)
pixels into a ROS‑style occupancy grid: 0 = free, 1 = obstacle.

Noise suppression:
  • global Otsu threshold (image is already quasi‑binary)
  • remove connected components smaller than --min-area px²
  • morphological closing to seal tiny pin‑holes on building edges
  • optional dilation (–inflate) to pad obstacles by the car’s footprint
"""
import cv2
import numpy as np
import argparse, yaml, os


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--input",  required=True,  help="Cropped screenshot (png/jpg/pgm)")
    p.add_argument("--output", required=True,  help="Binary PGM to write")
    p.add_argument("--yaml",   required=True,  help="Companion YAML for ROS nav stack")
    p.add_argument("--resolution", type=float, default=0.10, help="metres / pixel")
    p.add_argument("--min-area",   type=int,   default=200,  help="keep blobs ≥ this area")
    p.add_argument("--kernel",     type=int,   default=5,    help="closing kernel size")
    p.add_argument("--inflate",    type=int,   default=0,    help="extra px obstacle padding")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # --- 1) Load + greyscale --------------------------------------------------
    img_gray = cv2.imread(args.input, cv2.IMREAD_GRAYSCALE)
    if img_gray is None:
        raise FileNotFoundError(args.input)

    # --- 2) Global threshold (black = drivable) -------------------------------
    _, bin_mask = cv2.threshold(
        img_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    # In this screenshot: white = walls / ground returns, black = lidar hits (free).
    obst_mask = (bin_mask == 255).astype(np.uint8)

    # --- 3) Remove tiny speckle noise -----------------------------------------
    num, labels, stats, _ = cv2.connectedComponentsWithStats(obst_mask, connectivity=8)
    clean = np.zeros_like(obst_mask)
    for i in range(1, num):                       # label 0 is background
        if stats[i, cv2.CC_STAT_AREA] >= args.min_area:
            clean[labels == i] = 1

    # --- 4) Morphological closing to seal pin‑holes ---------------------------
    kernel = np.ones((args.kernel, args.kernel), np.uint8)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)

    # --- 5) Optional inflation (vehicle footprint / safety buffer) ------------
    if args.inflate > 0:
        kernel_inf = np.ones((args.inflate, args.inflate), np.uint8)
        clean = cv2.dilate(clean, kernel_inf)

    # --- 6) Write as binary PGM (0/1) -----------------------------------------
    pgm_data = (clean * 255).astype(np.uint8)     # keep native 8‑bit for viewers
    cv2.imwrite(args.output, pgm_data)

    # --- 7) Companion YAML (ROS map_server) -----------------------------------
    meta = {
        "image": os.path.basename(args.output),
        "resolution": args.resolution,
        "origin": [0.0, 0.0, 0.0],               # adjust if using global frame
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(args.yaml, "w") as f:
        yaml.safe_dump(meta, f)

    print(f"[✓] Occupancy grid saved to {args.output}")
    print(f"[✓] YAML metadata   saved to {args.yaml}")


if __name__ == "__main__":
    main()
