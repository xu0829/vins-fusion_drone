#!/usr/bin/env python3
"""看"重力方向在机身坐标系里"随时间转了多少 —— 判断 IMU 到底有没有被真实转动。

为什么需要它：陀螺读数可能被振动、时间轴错乱、数据损坏污染，"读数大"不等于
"真的转了"。加速度计的直流分量是重力的方向，它是**绝对参考**：静止不动时，
重力方向在机身系里是恒定的；真的把整个组件转了几十度，它会跟着转几十度。
两路证据一比：
  陀螺说转了很多、加速度计说重力方向没动 -> 陀螺读数不可信（振动/数据问题）；
  两路都说转了 -> 组件真的被转动过。

用法（容器内）: python3 gravity_direction_timeline.py <bag> [--from 0 --to 99]
"""
import sys

import numpy as np
import rosbag

args = sys.argv[1:]
lo, hi = 0.0, 100.0
if '--from' in args:
    lo = float(args[args.index('--from') + 1])
if '--to' in args:
    hi = float(args[args.index('--to') + 1])
bags = [a for a in args if a.endswith('.bag')]
if not bags:
    print(__doc__)
    sys.exit(1)

bag = rosbag.Bag(bags[0])
t_start = bag.get_start_time()
rows = []
for _, msg, _ in bag.read_messages(topics=['/imu']):
    t = msg.header.stamp.to_sec() - t_start
    if t < lo or t > hi:
        continue
    a, w = msg.linear_acceleration, msg.angular_velocity
    rows.append((t, np.array([a.x, a.y, a.z]), np.array([w.x, w.y, w.z]),
                 msg.header.stamp.to_sec()))
bag.close()
rows.sort()
if len(rows) < 50:
    print('IMU 样本不够')
    sys.exit(1)

t = np.array([r[0] for r in rows])
acc = np.array([r[1] for r in rows])
gyr = np.array([r[2] for r in rows])
dt = np.diff(t).clip(1e-4, 0.1)


def lpf(x, tau=1.0):
    """一阶低通，压掉振动，留下重力方向。"""
    y = np.empty_like(x)
    y[0] = x[0]
    alpha = dt[0] / (tau + dt[0])
    for i in range(1, len(x)):
        a = dt[i - 1] / (tau + dt[i - 1])
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


g = lpf(acc, tau=1.5)
g0 = g[0] / np.linalg.norm(g[0])

# 陀螺积分出来的姿态角（相对起始），用四元数累乘
q = np.array([1.0, 0, 0, 0])
q0 = q.copy()
ang_gyro = []
for i in range(len(t)):
    if i > 0:
        w = gyr[i - 1]
        th = np.linalg.norm(w) * dt[i - 1]
        if th > 1e-9:
            axis = w / np.linalg.norm(w)
            dq = np.concatenate([[np.cos(th / 2)], axis * np.sin(th / 2)])
            q = np.array([
                dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3],
                dq[0] * q[1] + dq[1] * q[0] + dq[2] * q[3] - dq[3] * q[2],
                dq[0] * q[2] - dq[1] * q[3] + dq[2] * q[0] + dq[3] * q[1],
                dq[0] * q[3] + dq[1] * q[2] - dq[2] * q[1] + dq[3] * q[0]])
    d = abs(float(np.dot(q, q0)))
    ang_gyro.append(np.degrees(2 * np.arccos(min(1.0, d))))
ang_gyro = np.array(ang_gyro)

gn = g / np.linalg.norm(g, axis=1, keepdims=True)
ang_acc = np.degrees(np.arccos(np.clip(gn @ g0, -1, 1)))

print(f'{bags[0].rsplit("/", 1)[-1]}   IMU 样本 {len(t)}   t={lo:.0f}~{hi:.0f}s')
print()
print(f'{"t(s)":>6} {"重力方向相对起始(度)":>22} {"陀螺积分相对起始(度)":>22} {"|a|均值":>9} {"陀螺中位":>10}')
print('-' * 74)
for s in range(int(lo), int(hi) + 1, 2):
    m = (t >= s) & (t < s + 2)
    if m.sum() < 5:
        continue
    print(f'{s:6d} {np.median(ang_acc[m]):22.2f} {np.median(ang_gyro[m]):22.2f} '
          f'{np.linalg.norm(acc[m], axis=1).mean():9.3f} {np.median(np.linalg.norm(gyr[m], axis=1)):10.4f}')
print()
print('判读：')
print('  两列都小且平稳      -> 组件基本没被转动')
print('  加速度计列大幅摆动  -> 组件真的被转动过（重力方向是绝对参考）')
print('  加速度计列平、陀螺列猛涨 -> 陀螺读数不可信，是振动/数据问题')
