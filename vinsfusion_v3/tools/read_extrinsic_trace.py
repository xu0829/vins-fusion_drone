#!/usr/bin/env python3
"""读 /vins_estimator/extrinsic 话题，看外参在整趟里到底有没有动、动到哪。

为什么需要它：extrinsic_parameter.csv 是 cv::FileStorage(WRITE) 整体重写的
（visualization.cpp:90），只留最后一帧的值，看不到过程，也看不到"有没有动"。
而 visualization.cpp:341-353 把同一个 ric[0]/tic[0] 发成 nav_msgs/Odometry
（position=tic[0] 米，orientation=ric[0]），按时间记下来就是完整过程。

用法（容器内）: python3 read_extrinsic_trace.py <bag> [话题]
"""
import math
import sys

import rosbag

bag_path = sys.argv[1]
topic = sys.argv[2] if len(sys.argv) > 2 else '/vins_estimator/extrinsic'

rows = []
with rosbag.Bag(bag_path) as bag:
    for _, msg, t in bag.read_messages(topics=[topic]):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rows.append((t.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))

if not rows:
    print(f'话题 {topic} 里一条消息都没有')
    print('（可能这趟 vins_node 还没进 NON_LINEAR，或者话题名不对）')
    sys.exit(1)

t0 = rows[0][0]
print(f'话题 {topic}')
print(f'样本 {len(rows)} 条，跨度 {rows[-1][0] - t0:.2f} s')
print()

# --- 平移 ---
print('=== tic[0] 平移（body 系下的 cam0 位置，单位 mm）===')
print(f'{"t(s)":>7} {"tx":>9} {"ty":>9} {"tz":>9}')
marks = [0, len(rows) // 10, len(rows) // 4, len(rows) // 2,
         3 * len(rows) // 4, len(rows) - 1]
for i in sorted(set(marks)):
    ts, x, y, z = rows[i][0] - t0, rows[i][1] * 1e3, rows[i][2] * 1e3, rows[i][3] * 1e3
    print(f'{ts:7.2f} {x:9.3f} {y:9.3f} {z:9.3f}')

xs = [r[1] for r in rows]
ys = [r[2] for r in rows]
zs = [r[3] for r in rows]
print()
print(f'全程取值范围  tx [{min(xs)*1e3:.3f}, {max(xs)*1e3:.3f}] mm')
print(f'              ty [{min(ys)*1e3:.3f}, {max(ys)*1e3:.3f}] mm')
print(f'              tz [{min(zs)*1e3:.3f}, {max(zs)*1e3:.3f}] mm')
span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
print(f'最大变动幅度  {span*1e3:.4f} mm')
if span < 1e-9:
    print('  -> 完全没有动（逐位相同）')
elif span * 1e3 < 0.01:
    print('  -> 基本没动（< 0.01 mm，只有浮点噪声）')
else:
    print('  -> 确实在动')
print()

# --- 旋转：相对第一帧的角度差 ---
print('=== ric[0] 旋转（相对第一帧的夹角，单位 度）===')
q0 = rows[0][4:8]


def quat_angle(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, d)))


angles = [quat_angle(r[4:8], q0) for r in rows]
print(f'{"t(s)":>7} {"夹角":>9}')
for i in sorted(set(marks)):
    print(f'{rows[i][0]-t0:7.2f} {angles[i]:9.4f}')
print()
print(f'全程最大旋转变动 {max(angles):.4f} 度')
if max(angles) < 1e-6:
    print('  -> 完全没有动（逐位相同）')
elif max(angles) < 0.01:
    print('  -> 基本没动（< 0.01 度）')
else:
    print('  -> 确实在动')
