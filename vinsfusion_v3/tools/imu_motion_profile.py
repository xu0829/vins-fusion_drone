#!/usr/bin/env python3
"""从 IMU 本身估计真实运动量：陀螺模长、加计偏离 g 的量。

用途：判断 t≈31 s 那个"速度越过 0.2 m/s"到底是
  (a) 相机真的开始被晃动了（那 0.2 是真运动，跑飞是跟踪失败），还是
  (b) 相机一直静止、0.2 是估计器自己涨上去的（那是零偏/尺度问题）。
这两种情况的修法完全不同，不能猜。

用法（容器内）: python3 imu_motion_profile.py <bag>
"""
import sys

import numpy as np
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path)
t0 = bag.get_start_time()

rows = []
for topic, msg, t in bag.read_messages(topics=['/imu']):
    a = msg.linear_acceleration
    w = msg.angular_velocity
    rows.append((msg.header.stamp.to_sec() - t0,
                 (w.x ** 2 + w.y ** 2 + w.z ** 2) ** 0.5,
                 (a.x ** 2 + a.y ** 2 + a.z ** 2) ** 0.5))
bag.close()

rows.sort()
arr = np.array(rows)
t = arr[:, 0]
gmag = arr[:, 1]
amag = arr[:, 2]

print(' 桶(s)   gyro中位(rad/s)  gyro最大   |a|中位    |a|标准差  偏离g最大')
print(' ' + '-' * 70)
for s in range(0, int(t[-1]) + 1, 5):
    m = (t >= s) & (t < s + 5)
    if m.sum() < 5:
        continue
    dev = np.abs(amag[m] - 9.80665)
    print(f'{s:5d}   {np.median(gmag[m]):13.4f} {gmag[m].max():9.4f} '
          f'{np.median(amag[m]):9.4f} {amag[m].std():10.4f} {dev.max():9.4f}')

print()
print('参照：静止手持时 gyro 中位约 0.02~0.03 rad/s（≈1.5°/s 的噪声底）')
print('      晃动时会到 0.5~3 rad/s；|a| 偏离 g 超过 0.5 m/s² 说明有真实加速度')
