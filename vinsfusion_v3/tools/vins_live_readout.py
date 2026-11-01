#!/usr/bin/env python3
"""实时读数：把 VINS 的位姿/速度/td 打在终端上，方便一边动一边看。

用途：验收"慢速平移/旋转，方向和尺度对不对"。
你按已知的方式动一下（比如沿桌子推 30cm、抬起来 20cm），看这里的数字对不对得上。

另一个关键作用：td 只在速度 ≥0.2 m/s 时才优化（estimator.cpp:1042），
所以静止时看它是没意义的。这个工具会把速度顶到 0.2 以上的时段标出来 ——
只有那些时刻 td 才真的在收敛。

用法: python3 vins_live_readout.py [秒数=60]
"""
import sys
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
TD_GATE = 0.2

rospy.init_node('vins_live_readout', anonymous=True, disable_signals=True)

last = {}
tds = []
n_fast = 0
n_tot = 0


def cb(m):
    global n_fast, n_tot
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    v = m.twist.twist.linear
    last['p'] = np.array([p.x, p.y, p.z])
    last['q'] = np.array([q.x, q.y, q.z, q.w])
    last['v'] = np.array([v.x, v.y, v.z])
    last['t'] = m.header.stamp.to_sec()
    last['n'] = last.get('n', 0) + 1

    sp = float(np.linalg.norm(last['v']))
    n_tot += 1
    if sp >= TD_GATE:
        n_fast += 1


rospy.Subscriber('/vins_estimator/odometry', Odometry, cb, queue_size=100)


def quat_to_rpy(q):
    x, y, z, w = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


p0 = None
print(f'监听 {DUR:.0f} 秒。现在开始动设备。')
print('（td 只在速度≥0.2 m/s 时才被优化，慢速平移/快转才有效）')
print()
print(f'{"t(s)":>6} {"x":>8} {"y":>8} {"z":>8} | {"|v|":>7} | '
      f'{"roll":>7} {"pitch":>7} {"yaw":>7} | td')
print('-' * 82)

t_start = time.time()
t_last_print = 0.0
while time.time() - t_start < DUR and not rospy.is_shutdown():
    time.sleep(0.05)
    if 'p' not in last:
        continue
    if p0 is None:
        p0 = last['p'].copy()
        t0 = last['t']
    el = last['t'] - t0
    if el - t_last_print < 0.5:
        continue
    t_last_print = el
    d = last['p'] - p0
    rpy = quat_to_rpy(last['q'])
    sp = float(np.linalg.norm(last['v']))
    mark = ' ←td在跑' if sp >= TD_GATE else ''
    print(f'{el:6.1f} {d[0]:8.3f} {d[1]:8.3f} {d[2]:8.3f} | {sp:7.3f} | '
          f'{rpy[0]:7.2f} {rpy[1]:7.2f} {rpy[2]:7.2f} |{mark}')

print()
if 'p' in last:
    d = last['p'] - p0
    print(f'总位移  Δx {d[0]:+.3f}  Δy {d[1]:+.3f}  Δz {d[2]:+.3f}  '
          f'|Δ| {np.linalg.norm(d):.3f} m')
    print(f'速度 ≥0.2 m/s 的样本：{n_fast}/{n_tot} '
          f'({100.0 * n_fast / max(1, n_tot):.1f}%)')
    if n_fast < 0.05 * max(1, n_tot):
        print('⚠️  几乎没有 ≥0.2 m/s 的时段 —— td 基本没被优化，动快一点')
    else:
        print('✅ 有足够快的时段，td 应该已经动过了 —— 看日志里的 td 值')
