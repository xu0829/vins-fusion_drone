#!/usr/bin/env python3
"""量两路红外流的**帧间隔分布** —— 判"链路有没有在饿着 VINS"。

为什么需要它：`rostopic hz` 只给平均频率，平均 30Hz 完全容得下"每 10 帧丢 1 帧"
或者"偶尔卡 200ms"。VINS 前端要的是**每一帧都按时到**，一次 200ms 的洞
就够让预积分和视觉接不上。

判据（实测基线，30fps ⇒ 标称间隔 0.0333s）：
    最大间隔 ≤ 0.05s     → 干净
    出现 > 0.09s 的洞    → 链路在饿 VINS，先查 USB
用法（容器内，需 source ROS）：
    python3 frame_gap.py [秒数]        默认 20
"""
import sys
import time

import rospy
from sensor_msgs.msg import Image

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TOPICS = [("/camera/infra1/image_rect_raw", "infra1"),
          ("/camera/infra2/image_rect_raw", "infra2")]

st = {n: [] for _, n in TOPICS}


def mk(name):
    def cb(msg):
        st[name].append(msg.header.stamp.to_sec())
    return cb


def main():
    rospy.init_node("frame_gap", anonymous=True)
    for tp, n in TOPICS:
        rospy.Subscriber(tp, Image, mk(n), queue_size=200)
    time.sleep(DUR)
    for _, n in TOPICS:
        t = st[n]
        if len(t) < 3:
            print("  %-7s 只收到 %d 帧 —— 话题可能没在发" % (n, len(t)))
            continue
        d = [t[i + 1] - t[i] for i in range(len(t) - 1)]
        ds = sorted(d)
        mean = sum(d) / len(d)
        p99 = ds[int(len(ds) * 0.99)]
        big = [x for x in d if x > 0.09]
        print("  %-7s %4d 帧 / %.0f 秒   平均间隔 %.4fs   最大 %.4fs   p99 %.4fs"
              % (n, len(t), DUR, mean, ds[-1], p99))
        print("           丢帧(间隔>0.09s)：%d 次%s"
              % (len(big), ("  最大洞 %.3fs" % max(big)) if big else "   ✓ 干净"))


if __name__ == "__main__":
    main()
