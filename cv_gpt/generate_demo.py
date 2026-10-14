#!/usr/bin/env python3
"""Create a synthetic image sequence, not a physical robot simulation."""
from pathlib import Path
import cv2
import numpy as np


def frame(bend=None):
    im = np.full((480, 640, 3), 230, dtype=np.uint8)
    if bend is not None:
        ys = np.arange(150, 480)
        xs = 320 + bend * 120*((480-ys)/330)**2
        pts = np.column_stack((xs, ys)).astype(np.int32)
        cv2.polylines(im, [pts], False, (20,20,20), 20)
    return im


def main():
    path = Path("synthetic_track.avi")
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (640,480))
    if not writer.isOpened():
        raise RuntimeError("MJPG video encoder unavailable")
    try:
        for seconds, bend in ((.8,0), (.8,1), (.4,None), (.8,1), (.8,-1), (4.5,None)):
            for _ in range(round(seconds*30)):
                writer.write(frame(bend))
    finally:
        writer.release()
    print(path.resolve())


if __name__ == "__main__":
    main()
