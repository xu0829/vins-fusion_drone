#!/usr/bin/env python3
"""半秒一格，把特征点数与 |v| / |p| 并排铺开，回答一件事：
  「t≈31 s 那一下，是点数先塌，还是速度先飞？」

为什么需要它：按秒（甚至按段）统计会把因果顺序抹平 —— 两个事件都落在 35~45 s
这一桶里，谁先谁后看不出来。而顺序正是唯一的判据：
  点数先塌、随后 |v| 才爬 -> 视觉约束先垮，IMU 独撑才导致速度失控；
  |v| 先爬、随后点数才塌 -> 位姿先错，预测投影像素偏了，跟踪才跟不上。

用法（容器内）: python3 points_vs_velocity.py <bag1> [<bag2> ...] [--from 26 --to 46]
对比两趟时把外参放开那趟放前面，锁死那趟放后面。
"""
import sys

import numpy as np
import rosbag

args = sys.argv[1:]
lo, hi = 26.0, 46.0
if '--from' in args:
    lo = float(args[args.index('--from') + 1])
if '--to' in args:
    hi = float(args[args.index('--to') + 1])
bags = [a for a in args if not a.startswith('--')
        and a not in (str(lo), str(hi))]


def load(path):
    t_pt, n_pt, t_od, p_od, v_od = [], [], [], [], []
    with rosbag.Bag(path) as bag:
        for topic, msg, _ in bag.read_messages(
                topics=['/vins_estimator/point_cloud', '/vins_estimator/odometry']):
            ts = msg.header.stamp.to_sec()
            if topic.endswith('point_cloud'):
                t_pt.append(ts)
                n_pt.append(len(msg.points))
            else:
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                t_od.append(ts)
                p_od.append([p.x, p.y, p.z])
                v_od.append([v.x, v.y, v.z])
    return (np.array(t_pt), np.array(n_pt, float),
            np.array(t_od), np.array(p_od), np.array(v_od))


if not bags:
    print(__doc__)
    sys.exit(1)

D = {}
for b in bags:
    lab = b.rsplit('/', 1)[-1].replace('.bag', '')
    D[lab] = load(b)

t0 = min(d[0][0] for d in D.values())

# 表头
hdr = f'{"t(s)":>6} |'
for lab in D:
    hdr += f' {lab[:26]:^27} |'
print(hdr)
sub = f'{"":>6} |'
for _ in D:
    sub += f' {"点数":>7}{"点数最小":>9}{"|v|":>8}{"|p|":>4} |'
print(sub)
print('-' * len(hdr))

for t in np.arange(lo, hi, 0.5):
    row = f'{t:6.1f} |'
    for lab, (t_pt, n_pt, t_od, p_od, v_od) in D.items():
        m = (t_pt - t0 >= t) & (t_pt - t0 < t + 0.5)
        mo = (t_od - t0 >= t) & (t_od - t0 < t + 0.5)
        if m.sum() == 0 or mo.sum() == 0:
            row += ' ' * 29 + '|'
            continue
        vn = np.linalg.norm(v_od[mo], axis=1)
        pn = np.linalg.norm(p_od[mo], axis=1)
        row += (f' {np.median(n_pt[m]):7.0f}{n_pt[m].min():9.0f}'
                f'{np.median(vn):8.3f}{np.median(pn):4.1f} |')
    print(row)
