#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""§1.3 坐标系验证：抬一次飞机，把三个 z 的符号和原点钉死。

为什么必须先做这一步：VINS 的 world 系、PX4 的 NED 系、以及 vision_uplink 在中间
做的 ENU/NED 转换，三者的 z 正方向、原点、甚至是否真的重力对齐都不一样。直接假设
"VINS z 向上、PX4 z 向下"然后混在一张表里比 Δ，坐标系错了是静默错，不报错。
本脚本只信实测：抬起来看谁变大谁变小。

用法（容器内）：
    python3 /root/xu_drone/live/z_frame_probe.py [总时长秒=40]

流程（终端会倒数提示，跟着走）：
     0 ~ 10 s   飞机静止放好，手离开
    10 ~ 15 s   垂直抬起约 0.3 m，保持住
    15 ~ 25 s   放回原位，手离开
    25 ~ 40 s   静止

产物：
    /tmp/zframe/<键>.csv     原始样本 stamp,arrival,x,y,z,roll,pitch,yaw
    /tmp/zframe/resampled.csv  50 Hz 最近邻重采样（跨话题对齐用）
    /tmp/zframe/verdict.txt  结论（供后续看门狗读取，不用再人工抄）
"""
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

OUT = "/tmp/vins_pi/zframe"

# 键, 话题, 类型, 该话题的 z 是否"向下为正"（仅作假设，结论以实测为准）
TOPICS = [
    ("vins_odom", "/vins_estimator/odometry", "odom", False),
    ("imu_prop", "/vins_estimator/imu_propagate", "odom", False),
    ("px4_ned", "/px4/local_position_ned", "pose", True),
]


def quat_to_rpy(q):
    """四元数 -> (roll, pitch, yaw)，度。手写以免依赖 tf。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def median(v):
    if not v:
        return float("nan")
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def spread(v):
    """返回 (p05, p50, p95)。"""
    if not v:
        return (float("nan"),) * 3
    s = sorted(v)
    pick = lambda f: s[min(len(s) - 1, max(0, int(f * (len(s) - 1))))]
    return pick(0.05), pick(0.50), pick(0.95)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    os.makedirs(OUT, exist_ok=True)

    data = {k: [] for k, _, _, _ in TOPICS}   # 每项: (stamp, arrival, x, y, z, r, p, yw)

    def make_cb(key, kind):
        def cb(msg):
            arrival = time.time()
            if kind == "odom":
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation
            else:
                pos = msg.pose.position
                ori = msg.pose.orientation
            r, p, yw = quat_to_rpy(ori)
            data[key].append((msg.header.stamp.to_sec(), arrival,
                              pos.x, pos.y, pos.z, r, p, yw))
        return cb

    rospy.init_node("z_frame_probe", anonymous=True)
    for key, topic, kind, _ in TOPICS:
        rospy.Subscriber(topic, Odometry if kind == "odom" else PoseStamped,
                         make_cb(key, kind), queue_size=2000)

    t0 = time.time()
    schedule = [
        (0.0,  "静止：飞机放好，手离开"),
        (10.0, ">>> 现在抬起：垂直抬约 0.3 m，保持住 <<<"),
        (15.0, ">>> 现在放下：放回原位，手离开 <<<"),
        (25.0, "静止：别碰"),
        (dur,  "结束"),
    ]
    step = 0
    print("总时长 %.0f s，跟着提示走。" % dur)
    while not rospy.is_shutdown() and time.time() - t0 < dur:
        el = time.time() - t0
        while step < len(schedule) and el >= schedule[step][0]:
            print("  [%5.1f s] %s" % (el, schedule[step][1]))
            step += 1
        time.sleep(0.05)

    # ---- 落盘原始 ----
    for key, _, _, _ in TOPICS:
        rows = data[key]
        with open(os.path.join(OUT, key + ".csv"), "w") as f:
            f.write("stamp,arrival,x,y,z,roll,pitch,yaw\n")
            for r in rows:
                f.write("%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n" % r)

    # ---- 50 Hz 最近邻重采样（跨话题对齐用）----
    grid = [t0 + i * 0.02 for i in range(int(dur / 0.02))]
    res = {k: [] for k in data}
    for k, rows in data.items():
        if not rows:
            res[k] = [None] * len(grid)
            continue
        arr = sorted(rows, key=lambda r: r[0])
        idx = 0
        for g in grid:
            while idx + 1 < len(arr) and abs(arr[idx + 1][0] - g) <= abs(arr[idx][0] - g):
                idx += 1
            res[k].append(arr[idx])
    with open(os.path.join(OUT, "resampled.csv"), "w") as f:
        f.write("t," + ",".join("%s_z,%s_up" % (k, k) for k, _, _, _ in TOPICS) + "\n")
        for i, g in enumerate(grid):
            cells = []
            for k, _, _, zdown in TOPICS:
                r = res[k][i]
                if r is None:
                    cells += ["", ""]
                else:
                    cells += ["%.5f" % r[4], "%.5f" % ((-r[4]) if zdown else r[4])]
            f.write("%.3f,%s\n" % (g - t0, ",".join(cells)))

    # ---- 分析 ----
    lines = []
    def emit(s=""):
        print(s)
        lines.append(s)

    emit()
    emit("=" * 74)
    emit("§1.3 坐标系验证结果")
    emit("=" * 74)
    emit()
    emit("%-11s %8s %8s %10s %10s   %s" % ("话题", "样本", "速率", "Δz(原始)", "Δup(翻转后)", "判定"))
    emit("-" * 74)

    verdicts = {}
    for key, topic, _, zdown in TOPICS:
        rows = data[key]
        if len(rows) < 20:
            emit("%-11s %8d  样本不足，跳过" % (key, len(rows)))
            continue
        ts = [r[0] for r in rows]
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if ts[-1] > ts[0] else 0.0
        base = median([r[4] for r in rows if r[0] - ts[0] < 8.0])
        devs = [(abs(r[4] - base), r) for r in rows]
        mag, peak = max(devs, key=lambda x: x[0])
        dz = peak[4] - base
        dup = -dz if zdown else dz
        verdict = "z 向上为正" if dz > 0 else "z 向下为正"
        if mag < 0.02:
            verdict = "没检测到抬起（阈值 2 cm）"
        verdicts[key] = (dz, dup, verdict)
        emit("%-11s %8d %7.1fHz %+10.4f %+11.4f   %s" % (key, len(rows), rate, dz, dup, verdict))

    emit()
    emit("=" * 74)
    emit("抬起向量 v_lift 与世界 z 轴的夹角（= 重力对齐误差估计，§4.4）")
    emit("=" * 74)
    for key, _, _, zdown in TOPICS:
        rows = data[key]
        if len(rows) < 20 or key not in verdicts:
            continue
        ts = [r[0] for r in rows]
        b = [median([r[i] for r in rows if r[0] - ts[0] < 8.0]) for i in (2, 3, 4)]
        devs = [(abs(r[4] - b[2]), r) for r in rows]
        _, peak = max(devs, key=lambda x: x[0])
        dv = [peak[2] - b[0], peak[3] - b[1], peak[4] - b[2]]
        if zdown:
            dv[2] = -dv[2]
        norm = math.sqrt(sum(c * c for c in dv))
        if norm < 0.02:
            continue
        ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(dv[2]) / norm))))
        emit("  %-11s |v_lift| = %6.4f m   水平分量 = %6.4f m   ->  夹角 %6.2f 度"
             % (key, norm, math.hypot(dv[0], dv[1]), ang))

    emit()
    emit("=" * 74)
    emit("传输时延（arrival - stamp，未标定 td 的可见部分）")
    emit("=" * 74)
    for key, _, _, _ in TOPICS:
        rows = data[key]
        if len(rows) < 20:
            continue
        d = [r[1] - r[0] for r in rows]
        p05, p50, p95 = spread(d)
        emit("  %-11s p05 %7.1f ms   p50 %7.1f ms   p95 %7.1f ms   离散度 %6.1f ms"
             % (key, p05 * 1e3, p50 * 1e3, p95 * 1e3, (p95 - p05) * 1e3))

    emit()
    emit("=" * 74)
    emit("姿态（静止段中位，用于排除姿态耦合）")
    emit("=" * 74)
    for key, _, _, _ in TOPICS:
        rows = [r for r in data[key] if r[0] - data[key][0][0] < 8.0] if data[key] else []
        if len(rows) < 20:
            continue
        emit("  %-11s roll %+7.3f   pitch %+7.3f   yaw %+8.3f  (度)"
             % (key, median([r[5] for r in rows]), median([r[6] for r in rows]),
                median([r[7] for r in rows])))

    # 两个 VINS 系之间的 roll/pitch 差 = VIO vs EKF 的姿态差
    if "vins_odom" in verdicts and "px4_ned" in verdicts:
        vo = [r for r in data["vins_odom"] if r[0] - data["vins_odom"][0][0] < 8.0]
        px = [r for r in data["px4_ned"] if r[0] - data["px4_ned"][0][0] < 8.0]
        if len(vo) > 20 and len(px) > 20:
            dr = median([r[5] for r in vo]) - median([r[5] for r in px])
            dp = median([r[6] for r in vo]) - median([r[6] for r in px])
            emit("  VIO - EKF 姿态差:  roll %+7.3f   pitch %+7.3f   合成 %6.3f 度"
                 % (dr, dp, math.hypot(dr, dp)))

    with open(os.path.join(OUT, "verdict.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print()
    print("已写 %s/{原始 CSV, resampled.csv, verdict.txt}" % OUT)


if __name__ == "__main__":
    main()
