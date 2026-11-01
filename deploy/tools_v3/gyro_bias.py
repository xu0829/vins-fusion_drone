#!/usr/bin/env python3
"""静止时直接量 /imu 的陀螺零偏真值 —— 用来对质 VINS 自己解出来的那个。

为什么必须单独量：VINS 启动时会打印
    gyroscope bias initial calibration  bx by bz
而它是**用视觉旋转反解**的（solveGyroscopeBias），视觉一脏就解出垃圾。
今晚香橙派这轮解出 bz=+0.00240，而记忆里飞控真值是 -0.0017 —— 符号都反了。
光看 VINS 的打印没法断定谁错，得有一个独立的真值。这就是那个真值。

用法（容器内，飞机必须静止）：
    python3 /root/xu_drone/live/gyro_bias.py [秒数]
"""
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Imu

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

rospy.init_node("gyro_bias", anonymous=True)
g, a = [], []
rospy.Subscriber("/imu", Imu,
                 lambda m: (g.append([m.angular_velocity.x, m.angular_velocity.y,
                                      m.angular_velocity.z]),
                            a.append([m.linear_acceleration.x, m.linear_acceleration.y,
                                      m.linear_acceleration.z])),
                 queue_size=200)
rospy.sleep(dur)

if len(g) < 50:
    print(f"  样本太少（{len(g)} 个），/imu 没在发？")
    sys.exit(1)

G = np.array(g)
A = np.array(a)
print(f"  样本 {len(G)} 个，历时 {dur:.0f} 秒")
print(f"  陀螺零偏真值 (rad/s): "
      f"x={G[:,0].mean():+.6f}  y={G[:,1].mean():+.6f}  z={G[:,2].mean():+.6f}")
print(f"              噪声 std: "
      f"x={G[:,0].std():.6f}  y={G[:,1].std():.6f}  z={G[:,2].std():.6f}")
print(f"  ⇒ 折算 yaw 漂移: {np.degrees(G[:,2].mean())*60:+.2f} °/分")
print(f"  加计均值 (m/s²): x={A[:,0].mean():+.4f}  y={A[:,1].mean():+.4f}  z={A[:,2].mean():+.4f}")
print(f"  |加计| = {np.linalg.norm(A.mean(axis=0)):.4f} m/s²  "
      f"(重力参考 9.805，偏差 {np.linalg.norm(A.mean(axis=0))-9.805:+.4f})")
print(f"  加计噪声 std: {A[:,0].std():.4f} {A[:,1].std():.4f} {A[:,2].std():.4f}")
