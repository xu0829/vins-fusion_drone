#!/usr/bin/env python3
"""并排比较多趟 estimate_extrinsic=1 跑出来的外参轨迹，回答：
  「它是不是每次都收敛到同一个答案？」

看三样东西：
  1. 各趟末值之间的离散度 —— 大 = 没收敛到同一个答案；
  2. 各趟【游走幅度】—— 比趟间离散度还大，说明它自己在跑，根本没稳定下来；
  3. 各趟相对初值的【偏移方向】是否一致 —— 一致 = 存在稳定的系统性偏差
     （这正是"能下手的地方"）；不一致 = 只是在噪声里乱走。

用法（容器内）: python3 compare_extrinsic_runs.py <bag1> <bag2> [bag3 ...]
"""
import sys

import numpy as np
import rosbag


def load(path):
    ts, tic, ric = [], [], []
    with rosbag.Bag(path) as bag:
        for _, msg, t in bag.read_messages(topics=['/vins_estimator/extrinsic']):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            ts.append(t.to_sec())
            tic.append([p.x, p.y, p.z])
            ric.append([q.x, q.y, q.z, q.w])
    return np.array(ts), np.array(tic), np.array(ric)


def qang(a, b):
    d = np.abs(np.sum(a * b, axis=-1))
    return np.degrees(2 * np.arccos(np.clip(d, -1, 1)))


names = sys.argv[1:]
if len(names) < 2:
    print(__doc__); sys.exit(1)

data = [load(p) for p in names]
if any(len(d[0]) == 0 for d in data):
    print('有 bag 里没有 /vins_estimator/extrinsic'); sys.exit(1)

init = data[0][1][0]
print(f'初值 (Kalibr)  tic = [{init[0]*1e3:.3f}, {init[1]*1e3:.3f}, {init[2]*1e3:.3f}] mm')
print()
print(f'{"趟":<26} {"末 tx":>9} {"末 ty":>9} {"末 tz":>9} {"相对初值偏移(mm)":>22} {"游走幅度(mm)":>13} {"转动(度)":>9}')
print('-' * 100)

finals, drifts, rots = [], [], []
for n, (ts, tic, ric) in zip(names, data):
    lab = n.rsplit('/', 1)[-1].replace('.bag', '')
    f = tic[-1]
    d = (f - init) * 1e3
    wander = (tic.max(axis=0) - tic.min(axis=0)) * 1e3
    rot = qang(ric, ric[0]).max()
    finals.append(f)
    drifts.append(d)
    rots.append(rot)
    print(f'{lab:<26} {f[0]*1e3:9.3f} {f[1]*1e3:9.3f} {f[2]*1e3:9.3f} '
          f'  ({d[0]:+7.2f},{d[1]:+7.2f},{d[2]:+7.2f}) {wander.max():13.2f} {rot:9.4f}')

finals = np.array(finals)
drifts = np.array(drifts)
print()
print('=== 结论 ===')
spread = (finals.max(axis=0) - finals.min(axis=0)) * 1e3
print(f'末值之间的离散度 (tx,ty,tz) = ({spread[0]:.2f}, {spread[1]:.2f}, {spread[2]:.2f}) mm')
print(f'参考：Kalibr 自身重复性约 7 mm；游走幅度远超这个数就说明没稳定下来')
print(f'各趟游走幅度最大 = {max((d[1].max(axis=0)-d[1].min(axis=0)).max()*1e3 for d in data):.2f} mm')
print()

print('相对初值的偏移方向：')
for i, ax in enumerate('xyz'):
    s = drifts[:, i]
    same = '一致' if (np.all(s > 0) or np.all(s < 0)) else '不一致（有正有负）'
    print(f'  t{ax}: {np.array2string(s, precision=2, floatmode="fixed"):>26} mm  -> {same}')
print()
if all((np.all(drifts[:, i] > 0) or np.all(drifts[:, i] < 0)) for i in range(3)):
    print('三轴偏移方向【全部一致】—— 这是稳定的系统性偏差，可以下手。')
else:
    print('偏移方向不一致的部分，说明那些轴只是在噪声里乱走。')
