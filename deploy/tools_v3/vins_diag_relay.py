#!/usr/bin/env python3
"""VINS/IMU 诊断转发节点 —— 新增文件，不改动任何现有代码。

为什么需要它：
  Foxglove 的 3D 面板只能给 nav_msgs/Path 画轨迹线（拖尾），
  nav_msgs/Odometry 只画一个坐标轴三脚架。VINS 只发布了 VIO 的 Path
  (/vins_estimator/path)，纯 IMU 那条 (/vins_estimator/imu_propagate)
  没有对应的 Path 话题 —— 所以在 3D 里看不到 IMU 的轨迹。
  本节点把两条 Odometry 都转成 Path，并顺带算出姿态诊断量。

发布的四个诊断话题：
  /diag/vio_path   nav_msgs/Path      VIO 轨迹（拖尾）
  /diag/imu_path   nav_msgs/Path      纯 IMU 递推轨迹（拖尾）
  /diag/vio_rpy    Vector3 (度)       VINS 的 roll / pitch / yaw
  /diag/grav_rp    Vector3 (度)       加计测的重力方向 roll / pitch，z=加速度模长
  /diag/att_delta  Vector3 (度)       VIO 与重力方向之差（滚转/俯仰/0）
  /diag/pos_norm   Vector3 (米)       x=|VIO pos|  y=|纯IMU pos|  z=两者之差

  ★ /diag/pos_norm 是判断"轨迹有没有飞走"最直观的一条线：
      静止时 x 和 y 都应该是 0.2 米上下的小常数。它一旦开始单向上扬，
      就是估计器发散了（不是真的在动 —— 位置差分速度会远大于上报速度）。

  ★ /diag/att_delta 是排查姿态问题的关键量：
      静止时应≈0。若它稳定停在几度上不归零，说明 VINS 卡在了
      "姿态倾斜 vs 加计零偏"的退化解上（静止时两者数学上不可区分），
      轻轻转动一下机体让两者可分，它就会归零。

用法（容器内）：
  source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
  python3 vins_diag_relay.py
"""

import math
import os
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu

VERBOSE = os.environ.get("DIAG_VERBOSE") == "1"

# 注意：/vins_estimator/imu_propagate 是在 IMU 回调里发布的（~150Hz），
# 不是 odometry 的 16Hz。所以阈值必须同时卡"时间"和"位移"，否则 Path 会瞬间膨胀。
MAX_POSES = 2000      # 每条 Path 最多保留的位姿数，防止消息无限膨胀
MIN_STEP_M = 0.010    # 距上一点超过 1cm 才追加
MIN_DT_S = 0.2        # 两次追加至少间隔 0.2s（实测踩过：只卡位移会攒到 2000+ 点）
HEARTBEAT_S = 2.0     # 即使没动，每 2 秒也追加一次，保证静止时轨迹不断
PUB_HZ = 1.0          # Path 发布频率（限流，别把 WiFi 打满）
ACC_WINDOW_S = 0.5    # 加计平均窗口（单次采样噪声 ±0.3°，平均后降到 0.05°）

_lock = threading.Lock()
_acc = []             # [(t, x, y, z)]
_vio_rp = {"roll": None, "pitch": None}
_vio_yaw = None
_last = {"vio": None, "imu": None}
_pos = {"vio": None, "imu": None}   # 最近一次位置，用来算 |pos|
_path = {"vio": Path(), "imu": Path()}
for p in _path.values():
    p.header.frame_id = "world"


def _rpy(x, y, z, w):
    """四元数 -> (roll, pitch, yaw)，单位度，ZYX 顺序。"""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _append(key, stamp, pos, quat):
    """按 (时间, 位移) 双阈值抽稀后追加到对应 Path。"""
    t = stamp.to_sec()
    prev = _last[key]
    if prev is not None:
        ppos, pt = prev
        dt = t - pt
        if dt < MIN_DT_S:
            return
        d = math.sqrt(sum((pos[i] - ppos[i]) ** 2 for i in range(3)))
        if d <= MIN_STEP_M and dt < HEARTBEAT_S:
            return
    _last[key] = (pos, t)

    ps = PoseStamped()
    ps.header.stamp = stamp
    ps.header.frame_id = "world"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
    ps.pose.orientation.x, ps.pose.orientation.y = quat[0], quat[1]
    ps.pose.orientation.z, ps.pose.orientation.w = quat[2], quat[3]
    p = _path[key]
    p.poses.append(ps)
    if len(p.poses) > MAX_POSES:
        del p.poses[:len(p.poses) - MAX_POSES]


def on_vio(msg):
    with _lock:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        global _vio_yaw
        r, pi, y = _rpy(q.x, q.y, q.z, q.w)
        _vio_rp["roll"], _vio_rp["pitch"], _vio_yaw = r, pi, y
        _pos["vio"] = (p.x, p.y, p.z)
        _append("vio", msg.header.stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def on_imu_prop(msg):
    with _lock:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        _pos["imu"] = (p.x, p.y, p.z)
        _append("imu", msg.header.stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def on_imu(msg):
    a = msg.linear_acceleration
    t = msg.header.stamp.to_sec()
    with _lock:
        _acc.append((t, a.x, a.y, a.z))
        cutoff = t - ACC_WINDOW_S
        while _acc and _acc[0][0] < cutoff:
            _acc.pop(0)


def main():
    # anonymous=True：重复启动时不会同名互相顶掉注册（踩过一次）
    #
    # disable_signals 必须留 False（默认）！踩过的坑：
    #   启动脚本用 `&` 在**非交互 shell** 里起后台任务，按 POSIX 规定，
    #   这类任务会被设成 SIGINT=SIG_IGN，而且这个"忽略"会 exec 继承给子进程。
    #   disable_signals=True 会让 rospy 不安装 SIGINT 处理器，于是那个继承来的
    #   SIG_IGN 一直生效 —— 这个节点就再也没法用 SIGINT 停掉了，
    #   脚本 cleanup 里的 pkill -INT 全部落空（表现为"停了半天还剩一个进程"）。
    #   rospy 默认（False）会装自己的处理器，SIGINT 才能正常触发 shutdown。
    rospy.init_node("vins_diag_relay", anonymous=True)

    pub_path = {
        "vio": rospy.Publisher("/diag/vio_path", Path, queue_size=1),
        "imu": rospy.Publisher("/diag/imu_path", Path, queue_size=1),
    }
    pub_vio_rpy = rospy.Publisher("/diag/vio_rpy", Vector3, queue_size=10)
    pub_grav_rp = rospy.Publisher("/diag/grav_rp", Vector3, queue_size=10)
    pub_delta = rospy.Publisher("/diag/att_delta", Vector3, queue_size=10)
    pub_posnorm = rospy.Publisher("/diag/pos_norm", Vector3, queue_size=10)

    rospy.Subscriber("/vins_estimator/odometry", Odometry, on_vio, queue_size=200)
    rospy.Subscriber("/vins_estimator/imu_propagate", Odometry, on_imu_prop, queue_size=200)
    rospy.Subscriber("/imu", Imu, on_imu, queue_size=5000)

    rospy.loginfo("[diag] 转发已启动：/diag/{vio_path,imu_path,vio_rpy,grav_rp,att_delta}")
    rate = rospy.Rate(PUB_HZ)
    if VERBOSE:
        print("[diag] 进入主循环 is_shutdown=%s" % rospy.is_shutdown(), flush=True)
    nloop = 0
    while not rospy.is_shutdown():
        nloop += 1
        if VERBOSE:
            print("[diag] 循环 %d: vio点数=%d imu点数=%d 加计样本=%d vio_roll=%s conns=%s" % (
                nloop, len(_path["vio"].poses), len(_path["imu"].poses), len(_acc),
                _vio_rp["roll"], [pub_path["vio"].get_num_connections(),
                                  pub_vio_rpy.get_num_connections()]), flush=True)
        with _lock:
            snap = {k: list(v.poses) for k, v in _path.items()}
            acc = list(_acc)
            rp = dict(_vio_rp)
            yaw = _vio_yaw
            pos = dict(_pos)
        for k, poses in snap.items():
            p = Path()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "world"
            p.poses = poses
            pub_path[k].publish(p)
        if rp["roll"] is not None:
            pub_vio_rpy.publish(Vector3(rp["roll"], rp["pitch"], yaw))
        if acc:
            n = len(acc)
            ax = sum(a[1] for a in acc) / n
            ay = sum(a[2] for a in acc) / n
            az = sum(a[3] for a in acc) / n
            mag = math.sqrt(ax * ax + ay * ay + az * az)
            g_roll = math.degrees(math.atan2(ay, az))
            g_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            pub_grav_rp.publish(Vector3(g_roll, g_pitch, mag))
            if rp["roll"] is not None:
                pub_delta.publish(Vector3(rp["roll"] - g_roll, rp["pitch"] - g_pitch, 0.0))
        if pos["vio"] is not None:
            nv = math.sqrt(sum(c * c for c in pos["vio"]))
            ni = nv if pos["imu"] is None else math.sqrt(sum(c * c for c in pos["imu"]))
            dd = 0.0 if pos["imu"] is None else math.sqrt(
                sum((pos["vio"][i] - pos["imu"][i]) ** 2 for i in range(3)))
            pub_posnorm.publish(Vector3(nv, ni, dd))
        rate.sleep()


if __name__ == "__main__":
    main()
