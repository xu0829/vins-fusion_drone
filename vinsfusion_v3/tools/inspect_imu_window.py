#!/usr/bin/env python3
"""细看某段 IMU：bag 到达时间 vs 消息头时间戳的偏差，找时钟事件。

为什么要分开看这两个时间：
  - VINS 用的是【消息头时间戳】(estimator.inputIMU(msg->header.stamp))，
    因为它要和图像头时间戳对齐；
  - rosbag play 是按【bag 到达时间】投递消息的。
  两者一旦不一致，投递节奏就和 VINS 认为的时间轴脱节 —— 成簇到达会让
  sync_process() 的 2ms 轮询有时拿得到、有时拿不到，进而触发
  processMeasurements() 里的 "wait for imu" 分支（!MULTIPLE_THREAD 时直接 return，
  把这一帧顺延到下一帧）。

用法（容器内）: python3 inspect_imu_window.py <bag> <起始秒> <结束秒>
"""
import sys

import numpy as np
import rosbag

bag_path = sys.argv[1]
t_lo = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
t_hi = float(sys.argv[3]) if len(sys.argv) > 3 else 37.0

bag = rosbag.Bag(bag_path)
t0 = bag.get_start_time()

rows = []
for topic, msg, t in bag.read_messages(topics=['/imu']):
    bt = t.to_sec() - t0
    if bt < t_lo - 2 or bt > t_hi + 2:
        continue
    rows.append((bt, msg.header.stamp.to_sec() - t0))
bag.close()

rows.sort()
bt = np.array([r[0] for r in rows])
hs = np.array([r[1] for r in rows])
off = hs - bt

print(f'窗口 {t_lo}~{t_hi} s，{len(rows)} 个 IMU 样本')
print(f'头时间戳 - bag到达时间 的偏差: 中位 {np.median(off)*1e3:.3f} ms  '
      f'min {off.min()*1e3:.3f}  max {off.max()*1e3:.3f} ms')
print()

print('  bag_t(s)   头戳(s)     偏差(ms)   dt头戳(ms)   dt到达(ms)')
print('  ' + '-' * 62)
for i in range(1, len(rows)):
    if bt[i] < t_lo or bt[i] > t_hi:
        continue
    d_h = (hs[i] - hs[i - 1]) * 1e3
    d_b = (bt[i] - bt[i - 1]) * 1e3
    # 只打印异常的：头戳间隔或到达间隔明显偏离 6.67ms 的
    if abs(d_h - 6.67) > 3.0 or abs(d_b - 6.67) > 3.0:
        print(f'  {bt[i]:9.4f} {hs[i]:10.4f} {off[i]*1e3:10.3f} {d_h:11.3f} {d_b:11.3f}')

print()
print('=== 每 0.5 秒的到达样本数（看投递是不是成簇）===')
for s in np.arange(t_lo, t_hi, 0.5):
    m = (bt >= s) & (bt < s + 0.5)
    bar = '#' * int(m.sum() / 2)
    print(f'  {s:6.1f}  {m.sum():4d}  {bar}')
