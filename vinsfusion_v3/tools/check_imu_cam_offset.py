#!/usr/bin/env python3
"""查 IMU 时间基准相对相机时间基准有没有发生过跳变。

思路：bag 到达时间对所有话题是同一个物理时钟，可以当公共参照。
  对每个话题算  (消息头时间戳 - bag到达时间)，
  这个量缓慢漂移是正常的（录制时队列延迟），但如果 IMU 的那条曲线
  相对图像的那条曲线【阶跃】了，说明 IMU 的时间轴被平移过 ——
  那 VINS 的 td 就整体错位，预积分出来的速度必然系统性偏。

这正是历史记录里"桥接把卡顿误判成时钟跳变、启动修复程序"会留下的痕迹，
而且能解释为什么"标定治不好"：错的不是外参，是时间轴。

用法（容器内）: python3 check_imu_cam_offset.py <bag>
"""
import sys

import numpy as np
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path)
t0 = bag.get_start_time()

imu_off, img_off = [], []
for topic, msg, t in bag.read_messages(
        topics=['/imu', '/camera/camera/infra1/image_rect_raw']):
    off = msg.header.stamp.to_sec() - t.to_sec()
    bt = t.to_sec() - t0
    (imu_off if topic == '/imu' else img_off).append((bt, off))
bag.close()

imu_off.sort()
img_off.sort()


def binned(rows, width=5.0):
    bt = np.array([r[0] for r in rows])
    off = np.array([r[1] for r in rows])
    out = []
    for s in np.arange(0, bt[-1], width):
        m = (bt >= s) & (bt < s + width)
        if m.sum() >= 3:
            out.append((s, np.median(off[m]) * 1e3))
    return out


bi = binned(imu_off)
bc = binned(img_off)
print(f'{"桶(s)":>8} {"IMU偏差(ms)":>13} {"图像偏差(ms)":>13} {"相对差(ms)":>12} {"相对差变化":>12}')
print('-' * 66)
prev = None
for (s, oi), (_, oc) in zip(bi, bc):
    rel = oi - oc
    delta = '' if prev is None else f'{rel - prev:+8.3f}'
    flag = ''
    if prev is not None and abs(rel - prev) > 1.0:
        flag = '  <== 阶跃'
    print(f'{s:8.0f} {oi:13.3f} {oc:13.3f} {rel:12.3f} {delta:>12}{flag}')
    prev = rel

print()
print('说明：相对差 = IMU偏差 - 图像偏差。它缓慢变化无所谓（录制队列延迟），')
print('      但【阶跃】意味着 IMU 时间轴相对相机被平移了。')
