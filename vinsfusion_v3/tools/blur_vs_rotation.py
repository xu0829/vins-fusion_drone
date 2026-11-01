#!/usr/bin/env python3
"""把每一帧的清晰度对上该帧时刻的陀螺转速，判断画面变糊是不是"运动模糊"。

为什么需要它：清晰度下降有两个完全不同的来源——
  (a) 运动模糊：曝光期间相机在转，像被抹开。特征提取会真的变差，只能靠
      降转速 / 缩曝光 / 提高跟踪鲁棒性来治；
  (b) 场景变化：相机转过去看到了没纹理的墙面。图像没糊，只是没用，
      换场景或加纹理就行。
两者的判据不同：把帧按陀螺转速分桶，若清晰度随转速单调下降，(a)；
若各转速桶的清晰度差不多，(b)。

用法（容器内）: python3 blur_vs_rotation.py <bag> [--from 26 --to 46]
"""
import sys

import cv2
import genpy
import numpy as np
import rosbag

args = sys.argv[1:]
lo, hi = 26.0, 46.0
if '--from' in args:
    lo = float(args[args.index('--from') + 1])
if '--to' in args:
    hi = float(args[args.index('--to') + 1])
bags = [a for a in args if a.endswith('.bag')]
if not bags:
    print(__doc__)
    sys.exit(1)
bag_path = bags[0]

IMG = '/camera/camera/infra1/image_rect_raw'

with rosbag.Bag(bag_path) as bag:
    t_start = bag.get_start_time()
    t_lo, t_hi = t_start + lo, t_start + hi

    # 先读 IMU：时间 -> 陀螺模长
    imu = []
    for _, msg, _ in bag.read_messages(topics=['/imu'],
                                       start_time=genpy.Time.from_sec(t_lo),
                                       end_time=genpy.Time.from_sec(t_hi)):
        w = msg.angular_velocity
        imu.append((msg.header.stamp.to_sec(), (w.x ** 2 + w.y ** 2 + w.z ** 2) ** 0.5))
    imu.sort()

    frames = []
    for _, msg, _ in bag.read_messages(topics=[IMG],
                                       start_time=genpy.Time.from_sec(t_lo),
                                       end_time=genpy.Time.from_sec(t_hi)):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        lap = cv2.Laplacian(img, cv2.CV_64F)
        frames.append((msg.header.stamp.to_sec(), float(lap.var())))

if not imu or not frames:
    print('没读到 IMU 或图像')
    sys.exit(1)

it = np.array([r[0] for r in imu])
iw = np.array([r[1] for r in imu])
ft = np.array([r[0] for r in frames])
fv = np.array([r[1] for r in frames])

# 每帧取最近一条 IMU（图像与 IMU 在同一个时钟上，直接最近邻即可）
idx = np.searchsorted(it, ft).clip(1, len(it) - 1)
w_at = np.where(np.abs(it[idx] - ft) < np.abs(it[idx - 1] - ft), iw[idx], iw[idx - 1])

print(f'{bag_path.rsplit("/", 1)[-1]}   t={lo}~{hi}s   帧数 {len(ft)}')
print()
print('按该帧的陀螺转速分桶：')
print(f'{"陀螺转速(rad/s)":>18} {"帧数":>6} {"清晰度中位":>12} {"清晰度均值":>12}')
print('-' * 52)
edges = [0, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0, 99]
for a, b in zip(edges[:-1], edges[1:]):
    m = (w_at >= a) & (w_at < b)
    if m.sum() == 0:
        continue
    print(f'{f"{a:g}~{b:g}":>18} {m.sum():6d} {np.median(fv[m]):12.1f} {fv[m].mean():12.1f}')

r = np.corrcoef(w_at, fv)[0, 1]
print()
print(f'清晰度 vs 陀螺转速 的相关系数 r = {r:+.3f}')
print('  r 显著为负 -> 转得越快画面越糊，是运动模糊 (a)')
print('  r 接近 0   -> 糊不随转速变，考虑场景/纹理原因 (b)')
