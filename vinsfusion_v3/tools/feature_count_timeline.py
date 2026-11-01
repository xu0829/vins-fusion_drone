#!/usr/bin/env python3
"""把一趟录制 bag 里的特征点数 / 位置 / 速度排成一条时间线，回答：
  「t≈31 s 之后到底是先跟丢、还是先跑飞？」

为什么需要它：/vins_estimator/point_cloud 的点数 = 当前被三角化、真正参与优化的
特征数，也就是"视觉约束还剩多少"。
  - 点数先塌、位置后飞  -> 视觉先垮，IMU 独撑才导致速度失控；
  - 点数一直很多、位置照样飞 -> 问题在 IMU 侧（零偏/尺度/时间轴）。
光看轨迹分不开这两种，因为两件事在时间上重叠。必须把点数与位置并排看。

用法（容器内）: python3 feature_count_timeline.py <bag> [<bag2> ...]
  单个 bag  -> 每秒一行的明细表；
  多个 bag  -> 先各印一段同一批时间桶的汇总，便于并排比。

注意：本脚本用消息 header 时间戳（= bag 里的图像时间），不是接收时间，
所以同一份 bag 的不同趟可以直接叠在一起比。
"""
import sys

import numpy as np
import rosbag

# 和之前手工分析用的同一批时间桶，便于和旧结论对齐
SEGMENTS = [(5, 15), (25, 30), (35, 45), (50, 95)]


def load(path):
    """取三样：点数、位置、速度。都用 header 时间戳。"""
    t_pt, n_pt, t_od, p_od, v_od = [], [], [], [], []
    with rosbag.Bag(path) as bag:
        for topic, msg, _ in bag.read_messages(
                topics=['/vins_estimator/point_cloud', '/vins_estimator/odometry']):
            ts = msg.header.stamp.to_sec()
            if topic.endswith('point_cloud'):
                t_pt.append(ts)
                n_pt.append(len(msg.points))
            else:
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                t_od.append(ts)
                p_od.append([p.x, p.y, p.z])
                v_od.append([v.x, v.y, v.z])
    return (np.array(t_pt), np.array(n_pt, dtype=float),
            np.array(t_od), np.array(p_od), np.array(v_od))


def label(path):
    return path.rsplit('/', 1)[-1].replace('.bag', '')


names = sys.argv[1:]
if not names:
    print(__doc__)
    sys.exit(1)

data = []
for n in names:
    d = (label(n),) + load(n)
    if len(d[1]) == 0:
        # 早期几趟录制只带了 extrinsic，没带 point_cloud/odometry，跳过并说清楚
        print(f'跳过 {d[0]}：bag 里没有 /vins_estimator/point_cloud')
        continue
    data.append(d)
if not data:
    print('所有 bag 里都没有 /vins_estimator/point_cloud —— 录制时没带上这个主题？')
    sys.exit(1)

# 所有 bag 共用一条时间轴，便于叠着看
t0 = min(d[1][0] for d in data)

for lab, t_pt, n_pt, t_od, p_od, v_od in data:
    print(f'===== {lab} =====')
    print(f'点数样本 {len(n_pt)} 条，跨度 {t_pt[-1] - t_pt[0]:.1f} s')
    print(f'{"t(s)":>7} {"点数中位":>9} {"点数最小":>9} {"|p|中位":>9} {"|v|中位":>9} {"|v|最大":>9}')
    print('-' * 60)

    def stat(lo, hi, arr_t, arr_v, fn):
        m = (arr_t - t0 >= lo) & (arr_t - t0 < hi)
        return fn(arr_v[m]) if m.sum() else np.nan

    pn = np.linalg.norm(p_od, axis=1)
    vn = np.linalg.norm(v_od, axis=1)

    for lo, hi in SEGMENTS:
        n = stat(lo, hi, t_pt, n_pt, np.median)
        nmin = stat(lo, hi, t_pt, n_pt, np.min)
        p = stat(lo, hi, t_od, pn, np.median)
        v = stat(lo, hi, t_od, vn, np.median)
        vmax = stat(lo, hi, t_od, vn, np.max)
        print(f'{lo:3d}~{hi:<3d} {n:9.1f} {nmin:9.1f} {p:9.3f} {v:9.4f} {vmax:9.4f}')
    print()

if len(data) > 1:
    print('===== 并排汇总（点数中位 / |p|中位 m / |v|中位 m·s⁻¹）=====')
    for lo, hi in SEGMENTS:
        print(f'--- {lo}~{hi} s ---')
        for lab, t_pt, n_pt, t_od, p_od, v_od in data:
            m_pt = (t_pt - t0 >= lo) & (t_pt - t0 < hi)
            m_od = (t_od - t0 >= lo) & (t_od - t0 < hi)
            if not m_pt.sum() or not m_od.sum():
                print(f'  {lab:<34} (无样本)')
                continue
            pn = np.linalg.norm(p_od[m_od], axis=1)
            vn = np.linalg.norm(v_od[m_od], axis=1)
            print(f'  {lab:<34} 点数 {np.median(n_pt[m_pt]):6.1f}   '
                  f'|p| {np.median(pn):7.3f}   |v| {np.median(vn):7.4f}')
