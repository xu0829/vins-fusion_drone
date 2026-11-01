#!/usr/bin/env python3
"""把 /imu 变成看得见的姿态角，专治"这只 IMU 到底飘不飘"。

为什么要单独一个节点：PX4 的 HIGHRES_IMU 里【没有姿态】(/imu 的
orientation_covariance[0] = -1，是"本消息不含姿态"的标准写法)。所以 Foxglove
里光看 /imu 是看不到 roll/pitch/yaw 的，只能看原始 xyz。

这里同时发三路姿态，故意分开算，因为它们飘起来的样子完全不同 —— 摆在一起看
就是最直接的判据：

  /imu/att_gyro_deg   纯陀螺积分。零偏没标干净的话，它【匀速地】越飘越远。
                      静止时这条线匀速爬升 = 陀螺零偏；抖 = 噪声大。
  /imu/att_accel_deg  纯加计测重力方向。没有积分，所以【永不累积误差】，
                      但瞬时很吵(运动加速度会污染它)。
                      它是"真值参考线"。
  /imu/rpy_deg        互补滤波 = 前两者按 0.98/0.02 融合。实际能用的姿态。

判读方法（静止放着别动）：
  · gyro 线和 accel 线【贴着走】  -> IMU 干净，可以放心用
  · gyro 线【匀速偏离】accel 线    -> 陀螺零偏，看偏离速率反推零偏大小
  · gyro 线【乱跳】                -> 噪声/丢包/时间戳有问题
  · accel 线自己【周期性摆动】     -> 设备被碰了，或者桌面在晃

yaw 是【没有绝对参考】的 —— 没有磁力计，yaw 只能靠陀螺积分，必然缓慢漂移。
这是物理限制，不是 bug。所以判据是看 roll/pitch，不是 yaw。

用法:
    python3 imu_rpy_node.py [话题=/imu]
"""
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import Imu

SRC = sys.argv[1] if len(sys.argv) > 1 else rospy.get_param('~src', '/imu')
ALPHA = float(rospy.get_param('~alpha', 0.98))   # 互补滤波：陀螺权重
G_NORM = 9.805

rospy.init_node('imu_rpy', anonymous=False)

pub_rpy = rospy.Publisher('/imu/rpy_deg', Vector3Stamped, queue_size=50)
pub_gyro = rospy.Publisher('/imu/att_gyro_deg', Vector3Stamped, queue_size=50)
pub_acc = rospy.Publisher('/imu/att_accel_deg', Vector3Stamped, queue_size=50)
pub_pose = rospy.Publisher('/imu/orientation', PoseStamped, queue_size=50)


def rpy_to_quat(r, p, y):
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class Rpy:
    def __init__(self):
        self.t_prev = None
        self.roll = self.pitch = self.yaw = 0.0      # 互补
        self.g_roll = self.g_pitch = self.g_yaw = 0.0  # 纯陀螺
        self.init = False
        # 静止零偏统计
        self.sum_g = np.zeros(3)
        self.sum_a = np.zeros(3)
        self.n = 0
        self.t_report = time.time()
        self.max_gyro_drift = 0.0
        self.t0 = None
        self.d_i = 0
        rospy.Subscriber(SRC, Imu, self.cb, queue_size=200)

    def cb(self, m):
        a = np.array([m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z])
        g = np.array([m.angular_velocity.x, m.angular_velocity.y,
                      m.angular_velocity.z])
        t = m.header.stamp.to_sec()

        if not self.init:
            # 用重力方向初始化 roll/pitch，避免开头一段瞎积分的垃圾
            self.roll, self.pitch = self._accel_rp(a)
            self.g_roll, self.g_pitch = self.roll, self.pitch
            self.t_prev = t
            self.t0 = t
            self.init = True
            return

        dt = t - self.t_prev
        self.t_prev = t
        if not (0 < dt < 0.5):
            return                      # 时间戳有洞，跳过这一步积分

        # ---- 纯陀螺积分（会飘的那条） ----
        self.g_roll += g[0] * dt
        self.g_pitch += g[1] * dt
        self.g_yaw += g[2] * dt

        # ---- 纯加计（不累积误差的参考线） ----
        a_roll, a_pitch = self._accel_rp(a)

        # ---- 互补滤波（实际能用的） ----
        self.roll = ALPHA * (self.roll + g[0] * dt) + (1 - ALPHA) * a_roll
        self.pitch = ALPHA * (self.pitch + g[1] * dt) + (1 - ALPHA) * a_pitch
        self.yaw += g[2] * dt

        hdr = m.header
        self.d_i += 1

        for pub, (x, y, z) in ((pub_rpy, (self.roll, self.pitch, self.yaw)),
                               (pub_gyro, (self.g_roll, self.g_pitch, self.g_yaw)),
                               (pub_acc, (a_roll, a_pitch, 0.0))):
            v = Vector3Stamped()
            v.header = hdr
            v.vector.x, v.vector.y, v.vector.z = np.degrees([x, y, z])
            pub.publish(v)

        ps = PoseStamped()
        ps.header = hdr
        ps.pose.orientation.x, ps.pose.orientation.y, \
            ps.pose.orientation.z, ps.pose.orientation.w = rpy_to_quat(
                self.roll, self.pitch, self.yaw)
        pub_pose.publish(ps)

        # ---- 静止零偏统计（只在看起来静止时累加） ----
        if np.linalg.norm(g) < 0.02 and abs(np.linalg.norm(a) - G_NORM) < 0.3:
            self.sum_g += g
            self.sum_a += a
            self.n += 1
        drift = abs(np.degrees(self.g_roll - a_roll))
        self.max_gyro_drift = max(self.max_gyro_drift, drift)

        self.report()

    @staticmethod
    def _accel_rp(a):
        ax, ay, az = a
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
        return roll, pitch

    def report(self):
        now = time.time()
        if now - self.t_report < 15.0:
            return
        el = now - self.t0
        if self.n > 100:
            bg = self.sum_g / self.n
            ba = self.sum_a / self.n
            # 陀螺零偏换算成"每小时飘多少度"
            deg_per_hr = np.degrees(np.linalg.norm(bg)) * 3600
            rospy.loginfo(
                '已跑 %.0fs；静止样本 %d 个。陀螺零偏 (%+.5f %+.5f %+.5f) rad/s '
                '= %.1f°/h；加计均值 (%+.4f %+.4f %+.4f) 模长 %.4f；'
                '陀螺积分 vs 加计 最大偏差 %.2f°',
                el, self.n, bg[0], bg[1], bg[2], deg_per_hr,
                ba[0], ba[1], ba[2], np.linalg.norm(ba), self.max_gyro_drift)
        else:
            rospy.loginfo('已跑 %.0fs；静止样本不足(%d)，判据要设备静放一会儿',
                          el, self.n)
        self.t_report = now


Rpy()
rospy.loginfo('监听 %s -> 发布 /imu/rpy_deg, /imu/att_gyro_deg, '
              '/imu/att_accel_deg, /imu/orientation', SRC)
rospy.spin()
