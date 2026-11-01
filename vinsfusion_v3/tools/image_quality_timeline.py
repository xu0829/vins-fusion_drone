#!/usr/bin/env python3
"""按时间桶统计图像本身的"清晰度"和亮度，回答：
  「t≈35.5 s 特征点数塌掉，是画面先糊了，还是画面一直好好的？」

为什么需要它：点数塌有两种完全不同的成因——
  (a) 图像侧：运动模糊/曝光变化 -> 特征根本提不出来 -> 前端跟丢；
  (b) 估计器侧：位姿预测错了 -> 光流初值偏 -> 匹配不上（图像其实是清楚的）。
两者的修法南辕北辙：(a) 调曝光/快门/降速，(b) 修 IMU/标定。
判据：Laplacian 方差（清晰度）若在点数塌之前就掉下去，是 (a)；
      若清晰度全程平稳而点数照样塌，是 (b)。

用法（容器内）: python3 image_quality_timeline.py <bag> [话题] [--from 26 --to 46]
默认话题 /camera/camera/infra1/image_rect_raw。
"""
import sys

import cv2
import genpy
import numpy as np
import rosbag

args = sys.argv[1:]
lo, hi, step = 26.0, 46.0, 1.0
for key, var in (('--from', 'lo'), ('--to', 'hi'), ('--step', 'step')):
    if key in args:
        val = float(args[args.index(key) + 1])
        if var == 'lo':
            lo = val
        elif var == 'hi':
            hi = val
        else:
            step = val
# 注意：bag 路径也以 '/' 开头，先按 .bag 摘出去，否则会被当成话题名（踩过）。
bags = [a for a in args if a.endswith('.bag')]
topics = [a for a in args if a.startswith('/') and not a.endswith('.bag')]
topic = topics[0] if topics else '/camera/camera/infra1/image_rect_raw'

if not bags:
    print(__doc__)
    sys.exit(1)
bag_path = bags[0]

# D430 的 IR 图是 16 位（mono16/16UC1），按 8 位解会 reshape 失败 —— 踩过。
DT = {'mono8': np.uint8, '8UC1': np.uint8, 'mono16': np.uint16, '16UC1': np.uint16}

rows = []
with rosbag.Bag(bag_path) as bag:
    t_start = bag.get_start_time()
    for _, msg, t in bag.read_messages(topics=[topic],
                                       start_time=genpy.Time.from_sec(t_start + lo),
                                       end_time=genpy.Time.from_sec(t_start + hi)):
        dt = DT.get(msg.encoding)
        if dt is None:
            print(f'不认识的编码 {msg.encoding}')
            sys.exit(1)
        img = np.frombuffer(msg.data, dtype=dt)
        if msg.step // np.dtype(dt).itemsize != msg.width:
            continue
        img = img.reshape(msg.height, msg.step // np.dtype(dt).itemsize)
        lap = cv2.Laplacian(img, cv2.CV_64F)
        scale = 1.0 if dt is np.uint8 else 255.0 / 65535.0
        rows.append((msg.header.stamp.to_sec(), float(lap.var()),
                     float(img.mean()) * scale,
                     float(np.percentile(img, 99)) * scale))

if not rows:
    print(f'{topic} 里没有可解码的图像')
    sys.exit(1)

rows.sort()
# 以 bag 起点为时间基准（--from/--to 也是 bag 相对秒），
# 这样才和外面按 bag 播放位置说的话对得上。
t0 = t_start
print(f'{topic}   帧数 {len(rows)}   跨度 {rows[-1][0] - t0:.1f} s')
print()
print(f'{"t(s)":>6} {"清晰度中位":>12} {"清晰度最小":>12} {"亮度中位":>10} {"p99":>7} {"帧数":>6}')
print('-' * 62)
s = lo
while s < hi:
    m = [r for r in rows if s <= r[0] - t0 < s + step]
    if not m:
        s += step
        continue
    lap = np.array([r[1] for r in m])
    mean = np.array([r[2] for r in m])
    p99 = np.array([r[3] for r in m])
    print(f'{s:6.1f} {np.median(lap):12.1f} {lap.min():12.1f} '
          f'{np.median(mean):10.1f} {np.median(p99):7.0f} {len(m):6d}')
    s += step
