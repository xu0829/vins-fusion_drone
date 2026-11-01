#!/usr/bin/env python3
"""盯住 att_delta 和 |pos| —— 直接验证"姿态跑掉 → 重力泄漏 → 发散"这条链。

机制：姿态估计偏了 θ 角，加计测到的重力就被错误地分解，水平方向凭空多出
g·sin(θ) 的假加速度，二次积分就是一条平滑的抛物线。
θ=5.5° 时泄漏 0.94 m/s²，355 秒能积出约 59 公里 —— 跟实测 28.6 公里同量级
（姿态误差是逐渐涨的，平均下来更小，所以实测偏小，自洽）。

判据（来自记忆 vins-attitude-runaway-divergence）：
  · 姿态跑掉 = att_delta **单向漂大**，不是来回摆
  · |pos| **平滑**地 ½at² 上扬（对比相机掉线是单帧阶跃）

/diag/pos_norm 的语义（见 vins_diag_relay.py:15-17）：
  x = |VIO 位置|   y = |纯 IMU 递推位置|   z = 两者之差
x、y 同时大 ⇒ 纯 IMU 那条也炸了 ⇒ 问题不只在视觉。

★ 单位坑（2026-09-28 亲身踩，务必记住）：
  /diag/att_delta **已经是"度"**（relay 里 rp["roll"] 和 g_roll 都由
  math.degrees() 产出，见 vins_diag_relay.py:64-69 和 178-185）。
  **绝不能再 math.degrees() 一次！** 我上一版就是这么写的，于是把真实的
  1.31° 印成了 75.2°，进而把"健康的姿态"误判成"姿态彻底跑飞"。
  教训同 [[instrument-must-touch-the-measured-signal]]：读数先跟原始话题对一遍。
"""
import math
import time

import rospy
from geometry_msgs.msg import Vector3

rospy.init_node("leak_watch", anonymous=True)
latest = {}


def mk(k):
    def cb(m):
        latest[k] = m
    return cb


rospy.Subscriber("/diag/att_delta", Vector3, mk("att"))
rospy.Subscriber("/diag/pos_norm", Vector3, mk("pos"))
rospy.sleep(2.0)

# 健康参考：静止时 ≈0.8°，退化到 2~6° 也常见；持续单向涨大才是病
print("   容器时(UTC)  已跑(s)   att_delta(°)  g·sin(θ)(m/s²)   |VIO|(m)   |纯IMU|(m)")
t0 = time.time()
for _ in range(9):
    a = latest.get("att")
    p = latest.get("pos")
    el = time.time() - t0
    if a is None or p is None:
        print(f"   {time.strftime('%H:%M:%S')}  {el:7.1f}   等数据...")
    else:
        # att_delta 的 x/y 本身就是"度"，直接取模长，不要再转一次
        deg = math.sqrt(a.x ** 2 + a.y ** 2)
        print(f"   {time.strftime('%H:%M:%S')}  {el:7.1f}   {deg:9.3f}   "
              f"{9.805 * math.sin(math.radians(deg)):12.4f}   "
              f"{p.x:9.4f}  {p.y:9.4f}")
    rospy.sleep(5.0)
