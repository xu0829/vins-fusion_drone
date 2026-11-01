#!/usr/bin/env python3
"""盯 /vins_estimator/imu_propagate 的**逐样本行为** —— 弄清"纯 IMU 轨迹"到底是什么。

背景：inputIMU() 每来一个 IMU 样本就 fastPredictIMU() 再
pubLatestOdometry(latest_P,...)，而 latest_P 由优化器重置。所以这条轨迹到底是
  (a) 每次视觉优化都被拉回 (那它就是"两次优化之间的短传播"，
      静止时应该始终贴着 0，偶尔的凸起 = 优化间隔)，还是
  (b) 从原点自由积分 (那它会单调漂走)？
决定"|纯IMU|"这个数该怎么读 —— 这直接关系到"视觉拉回"判据成不成立。

判据：
  相邻样本步长 ~ 亚毫米 且 位置在 0 附近小幅游走  → (a) 被反复拉回
  位置单调单向增长、步长稳定同号                  → (b) 自由积分

用法（容器内）：python3 prop_probe.py [秒数]   默认 20
"""
import sys
import time

import rospy
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
samples = []


def cb(m):
    p = m.pose.pose.position
    samples.append((m.header.stamp.to_sec(), p.x, p.y, p.z))


rospy.init_node("prop_probe", anonymous=True)
rospy.Subscriber("/vins_estimator/imu_propagate", Odometry, cb, queue_size=2000)
time.sleep(DUR)

if len(samples) < 10:
    print("  只收到 %d 个样本 —— 话题没在发？" % len(samples))
    raise SystemExit(0)

n = lambda s: (s[1] ** 2 + s[2] ** 2 + s[3] ** 2) ** 0.5
print("  %d 个样本 / %.0f 秒  (%.1f Hz)" % (len(samples), DUR, len(samples) / DUR))
mag = [n(s) for s in samples]
print("  |imu_propagate|  min %.4f  中位 %.4f  max %.4f  m" %
      (min(mag), sorted(mag)[len(mag) // 2], max(mag)))

# 相邻样本的位移（只算时间上相邻的，跳变会露出来）
steps = []
for i in range(len(samples) - 1):
    dt = samples[i + 1][0] - samples[i][0]
    if dt <= 0 or dt > 0.05:
        continue
    d = sum((samples[i + 1][k] - samples[i][k]) ** 2 for k in (1, 2, 3)) ** 0.5
    steps.append((d, dt))
if steps:
    ds = sorted(s[0] for s in steps)
    print("  相邻步长  min %.6f  中位 %.6f  p99 %.6f  max %.6f  m" %
          (ds[0], ds[len(ds) // 2], ds[int(len(ds) * 0.99)], ds[-1]))
    big = [s for s in steps if s[0] > 0.02]
    print("  步长 > 2cm 的跳变：%d 次" % len(big))
    if big:
        print("     ★ 有跳变 ⇒ 不是平滑传播，是【被优化器重置】的特征")
else:
    print("  样本间隔都异常，算不出步长")

print("\n  每 0.5 秒抽样（看是游走还是单调漂）：")
t0 = samples[0][0]
last = -1
for s in samples:
    k = int((s[0] - t0) / 0.5)
    if k != last:
        last = k
        print("    +%4.1fs  |p| = %8.4f m   (x %8.4f y %8.4f z %8.4f)"
              % (s[0] - t0, n(s), s[1], s[2], s[3]))
