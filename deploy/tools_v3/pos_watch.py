#!/usr/bin/env python3
"""按秒记录 VIO / 纯IMU / 视觉贡献 / 姿态差 / 原始IMU —— 一条 CSV，用来分辨"发散"还是"人在动"。

为什么非要它：今晚我用内联 awk 解析 rostopic echo，四层引号把字段号吃掉，
输出六行整齐的 0.000，我差点当成"飞机静止"记进结论。
落成文件跑就只有一层引号，不会再骗人。

判据（五条发散路径的指纹）
    |VIO| ≈ |纯IMU|（pos_norm 的 x≈y）  → 视觉约束归零
    att_delta 15~31° 且单帧阶跃          → 发射器污染
    att_delta 小 + |VIO| 平滑 ½at²       → 纯断流 / 特征饿死
    gyro 明显非零 + |v| 来回             → 人在动（不是发散）
    |v| 单调增                           → 发散

用法（容器内）：python3 /root/xu_drone/live/pos_watch.py [秒数] [CSV路径]
"""
import math
import sys
import time

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
csv_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/vins_pi/pos_watch.csv"

rospy.init_node("pos_watch", anonymous=True)

d = {}


def mk(k, f):
    def cb(m):
        d[k] = f(m)
    return cb


def odo(m):
    p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
    return (p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w)


def v3(m):
    return (m.x, m.y, m.z)


def imu(m):
    a, g = m.linear_acceleration, m.angular_velocity
    return (a.x, a.y, a.z, g.x, g.y, g.z)


rospy.Subscriber("/vins_estimator/odometry", Odometry, mk("vio", odo), queue_size=5)
rospy.Subscriber("/vins_estimator/imu_propagate", Odometry, mk("prop", odo), queue_size=5)
rospy.Subscriber("/diag/pos_norm", Vector3, mk("pn", v3), queue_size=5)
rospy.Subscriber("/diag/att_delta", Vector3, mk("ad", v3), queue_size=5)
rospy.Subscriber("/imu", Imu, mk("imu", imu), queue_size=20)

norm = lambda t: math.sqrt(sum(x * x for x in t[:3]))  # noqa: E731

with open(csv_path, "w") as fh:
    fh.write("t_wall,t_ros,vio_x,vio_y,vio_z,|vio|,|v_vio|,prop_x,prop_y,prop_z,"
             "|prop|,pn_x,pn_y,pn_z,ad_x,ad_y,ad_z,acc_norm,gyro_norm\n")
    t0 = time.time()
    print(f"  记录到 {csv_path}（{dur:.0f} 秒）", flush=True)
    print("   时刻     |VIO|   |纯IMU|  |VIO-IMU|  att_delta  |v|(m/s)  gyro   acc", flush=True)
    while not rospy.is_shutdown() and time.time() - t0 < dur:
        time.sleep(2.0)
        if "pn" not in d:
            print(f"   [{time.strftime('%H:%M:%S')}] 还没有 /diag/pos_norm", flush=True)
            continue
        pn = d["pn"]
        ad = d.get("ad", (float("nan"),) * 3)
        v = d.get("vio", (float("nan"),) * 10)
        pr = d.get("prop", (float("nan"),) * 10)
        im = d.get("imu", (float("nan"),) * 6)
        vv = norm(v[3:6]) if len(v) > 5 else float("nan")
        an = norm(im[0:3]) if len(im) > 2 else float("nan")
        gn = norm(im[3:6]) if len(im) > 5 else float("nan")
        print(f"   {time.strftime('%H:%M:%S')} {pn[0]:8.3f} {pn[1]:8.3f} {pn[2]:9.3f} "
              f"{ad[0]:8.2f}° {vv:8.3f} {gn:6.3f} {an:6.2f}", flush=True)
        fh.write(f"{time.time():.3f},{rospy.get_time():.6f},"
                 f"{v[0]:.6f},{v[1]:.6f},{v[2]:.6f},{norm(v[0:3]):.6f},{vv:.6f},"
                 f"{pr[0]:.6f},{pr[1]:.6f},{pr[2]:.6f},{norm(pr[0:3]):.6f},"
                 f"{pn[0]:.6f},{pn[1]:.6f},{pn[2]:.6f},"
                 f"{ad[0]:.6f},{ad[1]:.6f},{ad[2]:.6f},{an:.6f},{gn:.6f}\n")
        fh.flush()

print("  记录结束", flush=True)
