#!/usr/bin/env python3
"""Compare image header stamps with the matching realsense metadata stamps."""
import sys

import rosbag

PATH = sys.argv[1]
N = int(sys.argv[2]) if len(sys.argv) > 2 else 10

bag = rosbag.Bag(PATH, "r")


def head(topic, n):
    out = []
    for _t, msg, recv in bag.read_messages(topics=[topic]):
        h = getattr(msg, "header", None)
        st = h.stamp.to_sec() if h is not None else None
        seq = h.seq if h is not None else None
        out.append((st, seq, recv.to_sec(), msg))
        if len(out) >= n:
            break
    return out


for cam in ("infra1", "infra2"):
    img = head("/camera/%s/image_rect_raw" % cam, N)
    meta = head("/camera/%s/metadata" % cam, N)
    print("=== %s ===" % cam)
    print("image: n=%d first_stamp=%s seq=%s" % (len(img), img[0][0], img[0][1]) if img else "image: none")
    print("meta : n=%d first_stamp=%s seq=%s" % (len(meta), meta[0][0], meta[0][1]) if meta else "meta: none")
    if meta:
        m = meta[0][3]
        print("meta fields:", [a for a in dir(m) if not a.startswith("_")][:20])
        for attr in ("json_data", "data", "timestamp", "frame_number"):
            if hasattr(m, attr):
                print("  %s=%r" % (attr, getattr(m, attr)))
    # pair by order, show deltas
    print(" idx   image_stamp        meta_stamp         meta-image(ms)")
    for i in range(min(len(img), len(meta), N)):
        d = (meta[i][0] - img[i][0]) * 1e3 if (img[i][0] is not None and meta[i][0] is not None) else float("nan")
        print("%4d  %18.6f  %18.6f  %+10.3f" % (i, img[i][0], meta[i][0], d))

bag.close()
