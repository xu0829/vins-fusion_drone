#!/usr/bin/env python3
"""听 /imu 十几秒，告诉你这条流健不健康 —— 录包前先跑这个。

为什么需要：2026-09-27 实测发现 /imu 有【两种】看着都像"抖动"的东西，
但一个是无害的、一个是有害的，不分开看会误判：

  无害 —— 飞控时间戳的量化栅格。
      PX4 的 time_usec 有个固定栅格（实测 617us），所以相邻间隔会是
      8格/9格/10格 的整数倍摆动：4937 / 5554 / 6171 us。
      这些比值是 1、9/8、5/4 —— 小整数比。VINS 用真实 dt 积分，无害。

  有害 —— 真的丢消息。
      间隔跳成 2.33 倍这种【非整数比】（10514 / 4506），说明中间的消息
      根本没送到。丢 30% 的话 imu_utils 算出来的 Allan 方差是垃圾。

判据就是：长间隔是不是【短间隔的整数倍】。

用法:
    python3 imu_gap_check.py [时长秒=20] [话题=/imu]
"""
import sys
import time
from collections import Counter

import rospy
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/imu'

stamps = []


def cb(m):
    stamps.append(m.header.stamp.to_sec())


rospy.init_node('imu_gap_check', anonymous=True)
rospy.Subscriber(TOPIC, Imu, cb, queue_size=2000)
print('听 %s %.0f 秒…' % (TOPIC, DUR))
t_end = time.time() + DUR
while time.time() < t_end and not rospy.is_shutdown():
    time.sleep(0.2)

if len(stamps) < 100:
    print('只收到 %d 条 —— 话题没数据？' % len(stamps))
    sys.exit(1)

stamps.sort()
us = [round((b - a) * 1e6) for a, b in zip(stamps, stamps[1:])]
n = len(us)
span = stamps[-1] - stamps[0]
rate = n / span

print()
print('收到 %d 条，跨度 %.1f 秒 -> %.1f Hz' % (n, span, rate))
print()
print('=== 间隔分布（整数微秒，前 12 高）===')
cnt = Counter(us)
top = cnt.most_common(12)
base = top[0][0]
for v, c in sorted(top):
    r = v / base
    flag = ''
    if abs(r - round(r)) < 0.02 and round(r) >= 2:
        flag = '  <- %d 倍(量化，无害)' % round(r)
    elif v > base * 1.6:
        flag = '  <- 非整数倍(丢消息！)'
    print('  %6d us = %6.3f ms  x%-6d %5.1f%%  = %.3f x 基准%s'
          % (v, v / 1000, c, 100.0 * c / n, r, flag))

print()
print('=== 判读 ===')
short = [v for v, _ in top if v <= base * 1.35]
long_ = [v for v, c in cnt.items() if v > base * 1.35]
lost = 0
for v in long_:
    k = round(v / base)
    if abs(v / base - k) < 0.05 and k >= 2:
        lost += cnt[v] * (k - 1)          # 整数倍 -> 量化
bad = sum(c for v, c in cnt.items()
          if v > base * 1.35 and abs(v / base - round(v / base)) >= 0.05)
print('  基准周期 %.1f us (%.1f Hz)' % (base, 1e6 / base))
print('  整数倍长间隔（量化/正常）：%d 条' % lost)
print('  非整数倍长间隔（真丢包）  ：%d 条  (%.1f%%)'
      % (bad, 100.0 * bad / n))
print()
if bad < n * 0.01:
    print('  ✅ 健康 —— 可以录包')
elif bad < n * 0.10:
    print('  ⚠️  轻度丢包 —— 能跑 VIO，但别拿来标定')
else:
    print('  ❌ 严重丢包 —— 先查原因，别录 2 小时，imu_utils 会给出垃圾')
print()
print('  怎么查丢包原因：')
print('    1) 停掉 vins_node，再跑一次本脚本。若恢复正常 -> 是宿主 CPU 抢占了')
print('       px4_imu_node 的串口读取，串口缓冲溢出丢字节。')
print('    2) 停掉后依旧 -> 是飞控侧的问题（串口带宽/MAVLink 限流），')
print('       再查 PX4 的 MAV_x_RATE / 换个波特率试试。')
