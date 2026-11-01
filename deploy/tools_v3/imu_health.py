#!/usr/bin/env python3
"""直接盯住 /imu 和相机这两路信号本身 —— 不经过任何转发层。

用途：VINS 从干净原点 20 秒就发散、而相机又没掉线时，先确认
"喂进去的数据是不是坏的"。看四件事：

  1. 时间戳是否单调 —— 有没有重复或倒退。时间轴坏了 VINS 必炸
     （历史上有过：桥接误判时钟跳变，把 100 个样本压成同一时刻）。
  2. 相邻时间戳间隔的分布 —— 有没有大洞，或者挤成一团。
  3. 加计模长 —— 静止应稳定在 9.8 附近。大幅摆动说明飞机真在动，
     或者数据有尖刺。
  4. 相机帧间隔 —— 应精确 0.0333s。全程无超 0.09s 才是健康。
"""
import statistics
import sys

import rospy
from sensor_msgs.msg import Image, Imu

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

imu_t, acc, gyr = [], [], []
cam_t = []


def on_imu(m):
    imu_t.append(m.header.stamp.to_sec())
    a = m.linear_acceleration
    acc.append((a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5)
    g = m.angular_velocity
    gyr.append((g.x * g.x + g.y * g.y + g.z * g.z) ** 0.5)


def on_cam(m):
    cam_t.append(m.header.stamp.to_sec())


rospy.init_node("imu_health", anonymous=True)
rospy.Subscriber("/imu", Imu, on_imu, queue_size=5000)
rospy.Subscriber("/camera/infra1/image_rect_raw", Image, on_cam, queue_size=200)
rospy.sleep(dur)


def report(name, ts):
    if len(ts) < 3:
        print(f"  {name}: 样本太少（{len(ts)}）")
        return
    d = [b - a for a, b in zip(ts, ts[1:])]
    nonpos = sum(1 for x in d if x <= 0)
    print(f"  {name}: {len(ts)} 个样本, 实测 {len(ts) / dur:.2f} Hz")
    print(f"     间隔 min={min(d) * 1000:.3f}ms  中位={statistics.median(d) * 1000:.3f}ms"
          f"  max={max(d) * 1000:.3f}ms")
    print(f"     非正间隔(重复/倒退)={nonpos} 个    最大洞={max(d) * 1000:.1f}ms")
    print(f"     时间跨度={ts[-1] - ts[0]:.2f}s")


print("=== 直接盯信号本身（不经过转发层）===")
report("/imu", imu_t)
report("相机 infra1", cam_t)
if acc:
    print(f"  加计模长: min={min(acc):.3f}  中位={statistics.median(acc):.3f}"
          f"  max={max(acc):.3f} m/s²")
    print("      （静止应稳定在 9.8 附近；max 远大于 9.8 = 飞机真在动或有尖刺）")
if gyr:
    print(f"  陀螺模长: 中位={statistics.median(gyr):.5f}  max={max(gyr):.5f} rad/s")
