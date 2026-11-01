#!/usr/bin/env python3
"""按 10 秒分桶，看 IMU 头时间戳间隔的分布有没有随时间变化。

背景：单看整片会得到"dt 中位 4.547ms"这种被平均掉的结论。但 t≈31 s 是速度
越过 0.2 m/s、随后轨迹跑飞的时刻 —— 如果 IMU 的采样节奏恰好在这时变了，
那 VINS 的预积分就有系统性的时间尺度误差（这正是 t^4 增长律的候选解释）。

用法（容器内）: python3 imu_dt_histogram.py <bag>
"""
import sys

import numpy as np
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path)
t0 = bag.get_start_time()

hs, bt = [], []
for topic, msg, t in bag.read_messages(topics=['/imu']):
    hs.append(msg.header.stamp.to_sec() - t0)
    bt.append(t.to_sec() - t0)
bag.close()

hs = np.array(sorted(hs))
bt = np.array(sorted(bt))

print('=== 头时间戳间隔 dt（ms）按 10 秒分桶 ===')
print(f'{"桶(s)":>10} {"条数":>6} {"中位":>8} {"均值":>8} {"4.5ms占比":>10} {"10.5ms占比":>11} {"隐含频率":>9}')
print('-' * 72)
for s in range(0, int(bt[-1]) + 1, 10):
    m = (hs >= s) & (hs < s + 10)
    if m.sum() < 5:
        continue
    d = np.diff(hs[m]) * 1e3
    near45 = np.mean(np.abs(d - 4.5) < 0.5)
    near105 = np.mean(np.abs(d - 10.5) < 0.5)
    rate = 1000.0 / np.mean(d)
    print(f'{s:4d}-{s+10:<5d} {m.sum():6d} {np.median(d):8.3f} {d.mean():8.3f} '
          f'{near45:9.1%} {near105:10.1%} {rate:8.1f} Hz')

print()
print('=== 全片 dt 直方图 ===')
d = np.diff(hs) * 1e3
hist, edges = np.histogram(d, bins=np.arange(0, 30, 0.5))
for h, e in zip(hist, edges):
    if h > 0:
        print(f'  {e:5.1f}-{e+0.5:4.1f} ms  {h:7d}  {"#" * min(60, h // 200)}')
