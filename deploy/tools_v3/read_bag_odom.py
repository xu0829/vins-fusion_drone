#!/usr/bin/env python3
"""Compact VINS/PX4 time series from a bag (diagnostic)."""
import math
import sys

import rosbag

PATH = sys.argv[1]
T0 = 1792823965.0  # 14:39:25 local


def rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


bag = rosbag.Bag(PATH, "r")
last_print = -1e9
first_over = {}
stamps = []
print("=== /vins_estimator/odometry (1 s) ===")
print("  t     stamp        x        y        z      |pos|   roll  pitch  yaw")
for _topic, msg, _t in bag.read_messages(topics=["/vins_estimator/odometry"]):
    st = msg.header.stamp.to_sec()
    stamps.append(st)
    p = msg.pose.pose.position
    n = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
    for th in (1, 5, 10, 100, 1000):
        if n > th and th not in first_over:
            first_over[th] = (st, n)
    if st - last_print >= 1.0:
        last_print = st
        r, pi, ya = rpy(msg.pose.pose.orientation)
        print("%6.1f %12.3f %8.2f %8.2f %8.2f %8.2f %6.1f %6.1f %6.1f"
              % (st - T0, st, p.x, p.y, p.z, n, r, pi, ya))
d = [stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1)]
print("odom msgs %d, span %.1f s, dt p50 %.3f s max %.3f s, gaps>0.5s %d"
      % (len(stamps), stamps[-1] - stamps[0], sorted(d)[len(d) // 2], max(d),
         sum(1 for x in d if x > 0.5)))
for th in sorted(first_over):
    print("first |pos| > %5d m: t=%.3f (%.1f s), |pos|=%.1f" % (th, first_over[th][0], first_over[th][0] - T0, first_over[th][1]))

print()
print("=== /px4/local_position_ned (2 s) ===")
last_print = -1e9
for _topic, msg, _t in bag.read_messages(topics=["/px4/local_position_ned"]):
    st = msg.header.stamp.to_sec()
    if st - last_print >= 2.0:
        last_print = st
        p = msg.pose.position
        r, pi, ya = rpy(msg.pose.orientation)
        print("%6.1f %12.3f %8.3f %8.3f %8.3f   rpy %6.1f %6.1f %6.1f"
              % (st - T0, st, p.x, p.y, p.z, r, pi, ya))

for topic in ("/diag/pos_norm", "/diag/vio_rpy", "/diag/att_delta", "/diag/grav_rp"):
    print()
    print("=== %s ===" % topic)
    last_print = -1e9
    n = 0
    for _topic, msg, _t in bag.read_messages(topics=[topic]):
        n += 1
        try:
            st = msg.header.stamp.to_sec()
        except Exception:
            st = 0.0
        if st - last_print >= 2.0:
            last_print = st
            print("%6.1f %12.3f   %9.3f %9.3f %9.3f" % (st - T0, st, msg.x, msg.y, msg.z))
    print("msgs:", n)
bag.close()
