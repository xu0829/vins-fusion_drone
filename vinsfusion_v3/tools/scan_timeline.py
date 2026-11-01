#!/usr/bin/env python3
"""把 bag 里的 IMU 间隔、图像间隔、以及 VINS 输出的 |v|/位置，按时间排成一张表，
找 t≈31 s 那个"速度越过 0.2 然后跑飞"的触发点。

图像时间戳用 bag 的原始索引取（不解码图像），否则 2985 张 640x480 要解 900 MB。

用法（容器内）: python3 scan_timeline.py <bag> [轨迹csv]
"""
import sys

import numpy as np
import rosbag

bag_path = sys.argv[1]
csv_path = sys.argv[2] if len(sys.argv) > 2 else None

bag = rosbag.Bag(bag_path)
t0 = bag.get_start_time()

# --- 走连接索引拿时间戳，不解码图像（否则 2985 张 640x480 要解 900 MB）---
# _IndexEntry200 只有 chunk_pos/offset/position/time，没有 topic，
# 所以要从 _connections 拿到 conn_id -> topic 的映射再查 _connection_indexes。
topic_of = {cid: c.topic for cid, c in bag._connections.items()}
imu_ts, img1_ts = [], []
for cid, entries in bag._connection_indexes.items():
    tp = topic_of.get(cid, '')
    dst = imu_ts if tp == '/imu' else (img1_ts if tp.endswith('infra1/image_rect_raw') else None)
    if dst is None:
        continue
    dst.extend(e.time.to_sec() for e in entries)

imu_ts = np.array(sorted(imu_ts)) - t0
img1_ts = np.array(sorted(img1_ts)) - t0
bag.close()

print(f'imu {len(imu_ts)} 条, 图像 {len(img1_ts)} 张, 时长 {imu_ts[-1]:.1f} s')
print()

# --- 每秒一格的统计 ---
print(' bag_t(s)   IMU条数   dt中位(ms)  dt最大(ms)   图像  图像间隔中位(ms)')
print(' ' + '-' * 72)
secs = int(imu_ts[-1]) + 1
for s in range(secs):
    m = (imu_ts >= s) & (imu_ts < s + 1)
    if m.sum() < 2:
        continue
    d = np.diff(imu_ts[m]) * 1e3
    mi = (img1_ts >= s) & (img1_ts < s + 1)
    di = np.diff(img1_ts[mi]) * 1e3 if mi.sum() > 1 else np.array([np.nan])
    print(f'{s:8d} {m.sum():9d} {np.median(d):11.3f} {d.max():11.3f} {mi.sum():6d} {np.nanmedian(di):15.2f}')

# --- VINS 侧 ---
if csv_path:
    rows = []
    for ln in open(csv_path, errors='replace'):
        q = ln.strip().rstrip(',').split(',')
        if len(q) < 11:
            continue
        try:
            rows.append([float(x) for x in q[:11]])
        except ValueError:
            continue
    a = np.array(rows)
    if len(a) > 1:
        t = (a[:, 0] - a[0, 0]) / 1e9
        v = np.linalg.norm(a[:, 8:11], axis=1)
        p = np.linalg.norm(a[:, 1:4], axis=1)
        print()
        print(' --- VINS 输出：第一次 |v| > 0.2 附近 ---')
        k = int(np.argmax(v > 0.2)) if (v > 0.2).any() else -1
        if k > 0:
            print(f' 首次 |v|>0.2 在第 {k} 行, t={t[k]:.2f} s, |v|={v[k]:.4f} m/s')
            print(f' 前一帧 t={t[k-1]:.2f} s |v|={v[k-1]:.4f} |p|={p[k-1]:.4f}')
            print()
            print('     t(s)      |v|(m/s)     |p|(m)')
            for i in range(max(0, k - 40), min(len(a), k + 120), 10):
                print(f'  {t[i]:8.2f} {v[i]:12.4f} {p[i]:11.4f}')
