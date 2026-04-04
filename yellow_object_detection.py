#!/usr/bin/env python3
import os

# On Hyprland/Wayland, OpenCV's Qt windows often fail to load the Wayland plugin.
# Defaulting to XWayland ("xcb") makes cv2.imshow() work on most setups.
if os.environ.get("WAYLAND_DISPLAY") and not os.environ.get("QT_QPA_PLATFORM"):
    os.environ["QT_QPA_PLATFORM"] = "xcb"

import argparse
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class HsvRange:
    lower: Tuple[int, int, int]
    upper: Tuple[int, int, int]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Simple golden/yellow object segmentation + detection (OpenCV)."
    )
    src = p.add_mutually_exclusive_group(required=False)
    src.add_argument("--image", type=str, help="Path to an input image.")
    src.add_argument("--camera", type=int, default=0, help="Camera index (default: 0).")

    p.add_argument(
        "--hsv-low",
        type=int,
        nargs=3,
        default=(18, 90, 80),
        metavar=("H", "S", "V"),
        help="Lower HSV threshold (default: 18 90 80).",
    )
    p.add_argument(
        "--hsv-high",
        type=int,
        nargs=3,
        default=(38, 255, 255),
        metavar=("H", "S", "V"),
        help="Upper HSV threshold (default: 38 255 255).",
    )
    p.add_argument(
        "--min-area",
        type=int,
        default=800,
        help="Minimum contour area to count as an object (default: 800).",
    )
    p.add_argument(
        "--blur",
        type=int,
        default=5,
        help="Gaussian blur kernel size (odd). Set 0 to disable (default: 5).",
    )
    p.add_argument(
        "--morph",
        type=int,
        default=5,
        help="Morphology kernel size (odd). Set 0 to disable (default: 5).",
    )
    p.add_argument(
        "--show",
        action="store_true",
        help="Show debug windows (frame, mask, overlay).",
    )
    p.add_argument(
        "--no-gui",
        action="store_true",
        help="Disable all imshow() windows (useful on headless/Wayland Qt issues).",
    )
    p.add_argument(
        "--save-dir",
        type=str,
        default="yellow_out",
        help="Directory to save outputs when --no-gui (default: yellow_out).",
    )
    p.add_argument(
        "--save-every",
        type=int,
        default=30,
        help="Save every N frames when --no-gui on camera (default: 30).",
    )
    return p.parse_args()


def _odd_or_none(k: int) -> Optional[int]:
    if k <= 0:
        return None
    return k if (k % 2 == 1) else (k + 1)


def segment_yellow_bgr(frame_bgr: np.ndarray, hsv_range: HsvRange, blur_k: Optional[int], morph_k: Optional[int]) -> np.ndarray:
    if blur_k is not None:
        frame_bgr = cv2.GaussianBlur(frame_bgr, (blur_k, blur_k), 0)

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_range.lower, dtype=np.uint8), np.array(hsv_range.upper, dtype=np.uint8))

    if morph_k is not None:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    return mask


def find_objects(mask: np.ndarray, min_area: int):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    objs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        objs.append((area, (x, y, w, h), c))
    objs.sort(key=lambda t: t[0], reverse=True)
    return objs


def draw_detections(frame_bgr: np.ndarray, objs) -> np.ndarray:
    out = frame_bgr.copy()
    for i, (area, (x, y, w, h), contour) in enumerate(objs, start=1):
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.drawContours(out, [contour], -1, (0, 200, 255), 2)
        label = f"yellow #{i} area={int(area)}"
        cv2.putText(out, label, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
    return out


def run_on_image(path: str, hsv_range: HsvRange, blur_k: Optional[int], morph_k: Optional[int], min_area: int, show: bool) -> int:
    img = cv2.imread(path)
    if img is None:
        print(f"Failed to read image: {path}")
        return 2

    mask = segment_yellow_bgr(img, hsv_range, blur_k, morph_k)
    objs = find_objects(mask, min_area=min_area)
    vis = draw_detections(img, objs)

    if show:
        overlay = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow("frame", img)
        cv2.imshow("mask", mask)
        cv2.imshow("overlay", overlay)
        cv2.imshow("detections", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        out_path = "yellow_detections.png"
        cv2.imwrite(out_path, vis)
        print(f"Saved: {out_path} (objects: {len(objs)})")
    return 0


def run_on_camera(cam_index: int, hsv_range: HsvRange, blur_k: Optional[int], morph_k: Optional[int], min_area: int) -> int:
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"Failed to open camera index: {cam_index}")
        return 2

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                break

            mask = segment_yellow_bgr(frame, hsv_range, blur_k, morph_k)
            objs = find_objects(mask, min_area=min_area)
            vis = draw_detections(frame, objs)
            overlay = cv2.bitwise_and(frame, frame, mask=mask)

            cv2.imshow("detections (press q to quit)", vis)
            cv2.imshow("mask", mask)
            cv2.imshow("overlay", overlay)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


def run_on_camera_no_gui(
    cam_index: int,
    hsv_range: HsvRange,
    blur_k: Optional[int],
    morph_k: Optional[int],
    min_area: int,
    save_dir: str,
    save_every: int,
) -> int:
    import os
    os.makedirs(save_dir, exist_ok=True)

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"Failed to open camera index: {cam_index}")
        return 2

    frame_i = 0
    saved = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                break

            mask = segment_yellow_bgr(frame, hsv_range, blur_k, morph_k)
            objs = find_objects(mask, min_area=min_area)
            vis = draw_detections(frame, objs)

            if save_every > 0 and (frame_i % save_every == 0):
                base = os.path.join(save_dir, f"frame_{frame_i:06d}")
                cv2.imwrite(base + "_mask.png", mask)
                cv2.imwrite(base + "_detections.png", vis)
                saved += 1
                print(f"frame={frame_i} objects={len(objs)} saved={saved}")

            frame_i += 1
    finally:
        cap.release()

    print(f"Done. Saved {saved} frame(s) into: {save_dir}")
    return 0


def main() -> int:
    args = parse_args()
    hsv_range = HsvRange(lower=tuple(args.hsv_low), upper=tuple(args.hsv_high))
    blur_k = _odd_or_none(args.blur)
    morph_k = _odd_or_none(args.morph)

    if args.image:
        return run_on_image(
            args.image,
            hsv_range=hsv_range,
            blur_k=blur_k,
            morph_k=morph_k,
            min_area=args.min_area,
            show=(args.show and not args.no_gui),
        )
    if args.no_gui:
        return run_on_camera_no_gui(
            args.camera,
            hsv_range=hsv_range,
            blur_k=blur_k,
            morph_k=morph_k,
            min_area=args.min_area,
            save_dir=args.save_dir,
            save_every=args.save_every,
        )
    return run_on_camera(
        args.camera,
        hsv_range=hsv_range,
        blur_k=blur_k,
        morph_k=morph_k,
        min_area=args.min_area,
    )


if __name__ == "__main__":
    raise SystemExit(main())

