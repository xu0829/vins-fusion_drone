#!/usr/bin/env python3
"""体检一个正在发布的 /imu 话题：时间戳干不干净、坐标轴对不对。

这是"耳朵"能不能用的唯一判据。VINS 的 IMU 预积分假设 dt 准确，dt 脏了
再好的标定也救不回来。

三项必查：
  1. 时间戳在【宿主时钟域】（不是飞控开机秒）—— 映射失败会差 ~1e9 秒
  2. 单调递增、无"压扁"（dt≈0）—— 香橙派那套桥踩过的坑：100 个样本压成同一时刻
  3. 加速度模长 ≈ 9.8（静止）—— 顺带验证坐标轴与量纲

用法: python3 audit_imu_topic.py [秒数=15] [话题=/imu]
"""
import sys
import time

import numpy as np
import rospy
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/imu'

rospy.init_node('audit_imu_topic', anonymous=True, disable_signals=True)

stamps = []
accs = []


def cb(m):
    stamps.append(m.header.stamp.to_sec())
    a = m.linear_acceleration
    accs.append((a.x, a.y, a.z))


rospy.Subscriber(TOPIC, Imu, cb, queue_size=4000)
t0 = time.time()
while time.time() - t0 < DUR:
    time.sleep(0.05)

n = len(stamps)
if n < 100:
    print(f'❌ 只收到 {n} 条 —— 话题没在发，或发得太慢')
    sys.exit(1)

s = np.array(stamps)
wall = time.time()
print(f'收到 {n} 条，{n / DUR:.1f} Hz')
print()
print('【1】时钟域')
print(f'  首条时间戳 = {s[0]:.6f}')
print(f'  宿主当前   ≈ {wall:.6f}')
d = s[0] - wall
ok1 = abs(d) < 60
print(f'  差 = {d:+.3f} s   {"✅ 在宿主时钟域" if ok1 else "❌ 不在宿主域（映射失败）"}')

dt = np.diff(s)
print()
print('【2】时间戳质量')
print(f'  dt 中位 {np.median(dt) * 1e3:.3f} ms   均值 {dt.mean() * 1e3:.3f} ms   '
      f'标准差 {dt.std() * 1e3:.3f} ms')
print(f'  dt 最小 {dt.min() * 1e3:.4f} ms   最大 {dt.max() * 1e3:.3f} ms')
tiny = int((dt < 5e-4).sum())
big = int((dt > 5e-2).sum())
nonmono = int((dt <= 0).sum())
print(f'  压扁(dt<0.5ms) {tiny}   空洞(dt>50ms) {big}   非单调(dt<=0) {nonmono}')
ok2 = (nonmono == 0) and (tiny == 0)
print(f'  {"✅ 干净" if ok2 else "❌ 有问题"}')
print()
print('  dt 直方图 (ms):')
edges = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 20, 50, 1e9]
h, _ = np.histogram(dt * 1e3, bins=edges)
for i, c in enumerate(h):
    if c:
        bar = '#' * min(60, int(c / max(1, h.max()) * 60))
        lab = f'{edges[i]:5.1f}-{edges[i + 1]:5.1f}' if edges[i + 1] < 1e8 else f'{edges[i]:5.1f}+   '
        print(f'    {lab}  {c:5d}  {bar}')

a = np.array(accs)
mag = np.linalg.norm(a, axis=1)
print()
print('【3】加速度')
print(f'  模长中位 {np.median(mag):.3f} m/s²   (静止应≈9.8)')
print(f'  x {np.median(a[:, 0]):+7.3f}   y {np.median(a[:, 1]):+7.3f}   '
      f'z {np.median(a[:, 2]):+7.3f}')
ok3 = 8.5 < np.median(mag) < 11.0
print(f'  {"✅" if ok3 else "⚠️ 检查量纲/轴"}')

print()
if ok1 and ok2 and ok3:
    print('✅ 三项全过 —— 这只"耳朵"可以喂给 VINS 了')
else:
    print('❌ 有项没过，先别接 VINS')
