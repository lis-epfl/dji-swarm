"""
Synthetic DroneFeedSharedMemory publisher (no drones required).

Feeds fake frames into the "DroneFeedSharedMemory" mapping so the Unity DJI
scene (ImageSharing.cs) and the stitcher chain can be tested on the bench:
each fake drone gets a distinct heading and a labelled 800x450 BGR frame.

Usage (any python with cv2 + numpy):
    python image_feed_test.py                 # 4 drones at 0/45/90/135 deg
    python image_feed_test.py --drones 5
    python image_feed_test.py --headings 350 20 50 80

Expected in Unity (DJIScene) with StitcherThreading.py running:
  - one feed screen per drone, labelled and oriented by heading;
  - the stitcher receives the 3 headings closest to the pilot body yaw
    (enable_debug_logging dumps debug_input_drone_*.jpg to eyeball this);
  - Ctrl+C on one publisher run with --drones 2 -> ImageSharing stops
    re-publishing (<3 fresh feeds) and the panorama falls back to feeds.
"""

import argparse
import itertools
import mmap
import time

import cv2
import numpy as np

import utils.imageSharingUtil as imageSharingUtil

from image_stream_feed import (BLOCK_MAP_NAME, BLOCK_HEADER_BYTES, MAX_DRONES)

WIDTH, HEIGHT = 800, 450
IMAGE_BYTES = WIDTH * HEIGHT * 3
BLOCK_BYTES = BLOCK_HEADER_BYTES + IMAGE_BYTES


def make_frame(drone_id, heading, tick):
    """Distinctly coloured frame labelled with id/heading + a moving stripe."""
    hue = int(180 * (drone_id - 1) / max(1, MAX_DRONES - 1))
    hsv = np.full((HEIGHT, WIDTH, 3), (hue, 160, 120), dtype=np.uint8)
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    x = (tick * 7) % WIDTH   # motion so stitching/quality gates see change
    cv2.line(img, (x, 0), (x, HEIGHT), (255, 255, 255), 3)
    cv2.putText(img, "drone {}  {:.0f} deg".format(drone_id, heading),
                (40, HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                (255, 255, 255), 3)
    return img


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drones", type=int, default=4)
    parser.add_argument("--headings", type=float, nargs="*", default=None,
                        help="one heading per drone (deg); default 45-deg fan")
    parser.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()

    if not 1 <= args.drones <= MAX_DRONES:
        parser.error("--drones must be 1..{}".format(MAX_DRONES))
    headings = (args.headings if args.headings is not None
                else [45.0 * i for i in range(args.drones)])
    if len(headings) != args.drones:
        parser.error("need exactly one heading per drone")

    mmf = mmap.mmap(-1, MAX_DRONES * BLOCK_BYTES, BLOCK_MAP_NAME)
    print("Publishing {} fake drones to '{}' at {:.0f} fps (Ctrl+C to stop)"
          .format(args.drones, BLOCK_MAP_NAME, args.fps))

    try:
        for tick in itertools.count():
            for i, heading in enumerate(headings):
                drone_id = i + 1
                img = make_frame(drone_id, heading, tick)
                imageSharingUtil.write_memory(
                    mmf, (drone_id - 1) * BLOCK_BYTES, IMAGE_BYTES, img,
                    drone_id - 1, heading, pace_s=0.0)
            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        pass
    finally:
        mmf.close()


if __name__ == "__main__":
    main()
