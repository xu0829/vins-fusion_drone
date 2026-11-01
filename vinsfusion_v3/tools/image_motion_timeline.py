#!/usr/bin/env python3
"""量相邻帧之间场景在画面里移动了多少像素 —— 直接判断"相机到底动没动"。

为什么需要它：陀螺仪读数只说明"IMU 感受到了角速度"，不说明相机真的转了
（振动、数据损坏、时间轴错乱都能让陀螺读数变大）。而画面位移是被测物体的
物理事实：相机真转了，画面内容就会整体平移；只是被振动糊了，内容位置基本不动。
这是分"真的在晃"和"只是读数在晃"的唯一直接证据，别的都是推理。

参照量级：f≈380 px 时，0.36 rad/s ≈ 137 px/s ≈ 4.6 px/帧；
          真静止时相邻帧位移应 < 0.5 px。

用法（容器内）: python3 image_motion_timeline.py <bag> [--from 26 --to 46]
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

IMG = '/camera/camera/infra1/image_rect_raw'

stamps, imgs = [], []
with rosbag.Bag(bags[0]) as bag:
    t_start = bag.get_start_time()
    for _, msg, _ in bag.read_messages(
            topics=[IMG],
            start_time=genpy.Time.from_sec(t_start + lo),
            end_time=genpy.Time.from_sec(t_start + hi)):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        stamps.append(msg.header.stamp.to_sec())
        imgs.append(a.astype(np.float32))

if len(imgs) < 3:
    print('帧数不够')
    sys.exit(1)

t0 = stamps[0]
win = cv2.createHanningWindow((imgs[0].shape[1], imgs[0].shape[0]), cv2.CV_32F)

pair = []      # (t, 相邻帧位移 px, 相位相关峰值)
sec = []       # (t, 相隔约 1 s 的两帧位移 px)
for i in range(1, len(imgs)):
    (dx, dy), resp = cv2.phaseCorrelate(imgs[i], imgs[i - 1], win)
    pair.append((stamps[i] - t0, float(np.hypot(dx, dy)), float(resp)))

j = 0
for i in range(len(imgs)):
    while j + 1 < i and stamps[i] - stamps[j + 1] >= 1.0:
        j += 1
    if i > j and stamps[i] - stamps[j] >= 0.9:
        (dx, dy), resp = cv2.phaseCorrelate(imgs[i], imgs[j], win)
        sec.append((stamps[i] - t0, float(np.hypot(dx, dy))))

print(f'{bags[0].rsplit("/", 1)[-1]}   帧数 {len(imgs)}   t={lo}~{hi}s')
print()
print(f'{"t(s)":>6} {"相邻帧位移中位(px)":>20} {"最大":>8} {"峰值中位":>10} {"1s 基线位移中位(px)":>20}')
print('-' * 70)
# pair/sec 里的时间是"窗口内秒数"（t0 = 窗口第一帧），分桶要加回 lo
for s in range(int(lo), int(hi)):
    a = [r for r in pair if s - lo <= r[0] < s - lo + 1]
    b = [r for r in sec if s - lo <= r[0] < s - lo + 1]
    if not a:
        continue
    d = np.array([r[1] for r in a])
    r_ = np.array([r[2] for r in a])
    ds = np.median([r[1] for r in b]) if b else float('nan')
    print(f'{s:6d} {np.median(d):20.3f} {d.max():8.3f} {np.median(r_):10.3f} {ds:20.2f}')
print()
print('判读：相邻帧位移 ~0 px 且 1s 基线也 ~0 px -> 相机没动；')
print('      相邻帧几 px、1s 基线几十~上百 px -> 相机真的在转/在晃。')
