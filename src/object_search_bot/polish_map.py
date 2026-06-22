#!/usr/bin/env python3
import argparse
import cv2
import numpy as np
import os


def flood_fill_clean_map(input_path, output_path, kernel_size=5):
    if not os.path.exists(input_path):
        print(f"❌ Error: Input file '{input_path}' not found.")
        return

    print(f"📖 Loading noisy map asset: {input_path}")
    img = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape

    # 1. Extract walls as WHITE (255) on a BLACK (0) background
    # This ensures morphology patches the walls instead of eating them!
    wall_mask = np.zeros_like(img)
    wall_mask[img < 50] = 255

    # 2. Morphological closing to seal ALL wall fractures securely
    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (kernel_size, kernel_size)
    )
    print(f"🛠️ Sealing wall gaps with kernel size: {kernel_size}x{kernel_size}")
    closed_walls = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, kernel)

    # 3. Setup our Flood Fill canvas (Default background = ROS Unknown Gray)
    polished_map = np.full_like(img, 205)

    # 4. Find a guaranteed open floor pixel inside the room to drop the paint bucket
    seed_point = None
    for y in range(int(h * 0.5), int(h * 0.9)):
        for x in range(int(w * 0.3), int(w * 0.7)):
            # Look for an original white pixel that isn't overridden by a closed wall
            if img[y, x] > 240 and closed_walls[y, x] == 0:
                seed_point = (x, y)
                break
        if seed_point:
            break

    if seed_point is None:
        seed_point = (int(w / 2), int(h / 2))

    print(f"🌊 Flooding map interior from safe floor coordinate: {seed_point}")

    # Use the closed walls as an absolute boundary barrier
    flood_canvas = closed_walls.copy()
    mask = np.zeros((h + 2, w + 2), np.uint8)

    # Flood fill the walkable interior room space with a temporary marker (128)
    cv2.floodFill(flood_canvas, mask, seed_point, 128)

    # 5. Reconstruct the clean ROS trinary map matrix
    # Everywhere the fluid traveled becomes pure white Free Space
    polished_map[flood_canvas == 128] = 255

    # Everywhere the patched walls are becomes solid sharp Black walls
    polished_map[closed_walls == 255] = 0

    # Clean up any leftover stray loose pixel specks inside the rooms
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        (polished_map == 0).astype(np.uint8), connectivity=8
    )
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] < 3:
            polished_map[labels == i] = 255

    # Save clean asset
    cv2.imwrite(output_path, polished_map)
    print(f"🎉 Success! Strictly closed map exported cleanly to: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Prunes maps down to pristine vector layouts."
    )
    parser.add_argument("-i", "--input", required=True, help="Path to raw map")
    parser.add_argument(
        "-o", "--output", help="Path to output clean map destination"
    )
    parser.add_argument(
        "-k",
        "--kernel",
        type=int,
        default=5,
        help="Wall sealing kernel size (default: 5)",
    )
    args = parser.parse_args()

    if not args.output:
        base, ext = os.path.splitext(args.input)
        args.output = f"{base}_clean{ext}"

    flood_fill_clean_map(args.input, args.output, kernel_size=args.kernel)