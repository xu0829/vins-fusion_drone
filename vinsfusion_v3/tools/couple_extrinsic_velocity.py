#!/usr/bin/env python3
"""把外参 tic[0] 和速度 |v| 按时间对齐，看它俩是不是一起跑飞的。

动机：estimator.cpp:1029 的放开条件是 Vs[0].norm() > 0.2 —— 也就是"速度超过
0.2 m/s 才允许优化外参"。如果外参恰好在这条线被跨过时开始动，说明两者是
同一个失稳回路里的两个量，而不是"外参标定错了导致速度跑飞"这种单向因果。

用法（容器内）: python3 couple_extrinsic_velocity.py <bag>
"""
import sys

import rosbag

bag_path = sys.argv[1]

ext, odo = [], []
with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(topics=['/vins_estimator/extrinsic',
                                                   '/vins_estimator/odometry']):
        ts = t.to_sec()
        if topic.endswith('extrinsic'):
            p = msg.pose.pose.position
            ext.append((ts, p.x, p.y, p.z))
        else:
            v = msg.twist.twist.linear
            ext_ = msg.pose.pose.position
            odo.append((ts, (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5,
                        (ext_.x ** 2 + ext_.y ** 2 + ext_.z ** 2) ** 0.5))

if not ext or not odo:
    print('话题不全'); sys.exit(1)

t0 = min(ext[0][0], odo[0][0])
ext = [(t - t0, x, y, z) for t, x, y, z in ext]
odo = [(t - t0, v, p) for t, v, p in odo]

print('  t(s)   |v|(m/s)  位置(m)   tx(mm)   ty(mm)   tz(mm)   外参变动(mm)')
print('  ' + '-' * 68)

prev = None
started = None
for i in range(0, len(ext), max(1, len(ext) // 22)):
    te, x, y, z = ext[i]
    # 找时间最近的速度样本
    j = min(range(len(odo)), key=lambda k: abs(odo[k][0] - te))
    _, v, p = odo[j]
    if prev is None:
        d = 0.0
    else:
        d = ((x - prev[0]) ** 2 + (y - prev[1]) ** 2 + (z - prev[2]) ** 2) ** 0.5 * 1e3
    prev = (x, y, z)
    if started is None and d > 0.5:
        started = te
    print(f'{te:7.2f}  {v:8.4f}  {p:7.3f}  {x*1e3:8.2f} {y*1e3:8.2f} {z*1e3:8.2f}   {d:9.3f}')

print()
# 第一次速度越过 0.2 的时刻
cross = next((t for t, v, _ in odo if v > 0.2), None)
print(f'速度首次 > 0.2 m/s 的时刻: {"%.2f s" % cross if cross else "从未超过"}')
print(f'外参首次明显变动的时刻: {"%.2f s" % started if started else "从未变动"}')
if cross and started:
    print(f'两者相差 {started - cross:+.2f} s')
