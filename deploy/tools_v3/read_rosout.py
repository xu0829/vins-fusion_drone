#!/usr/bin/env python3
"""Print /rosout messages from a bag after a given stamp (diagnostic)."""
import sys

import rosbag

path = sys.argv[1]
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
bag = rosbag.Bag(path, "r")
try:
    for _topic, msg, _t in bag.read_messages(topics=["/rosout"]):
        ts = msg.header.stamp.to_sec()
        if ts >= t0:
            text = msg.msg.replace("\n", " | ")
            print("%.3f L%d %s: %s" % (ts, msg.level, msg.name, text))
finally:
    bag.close()
