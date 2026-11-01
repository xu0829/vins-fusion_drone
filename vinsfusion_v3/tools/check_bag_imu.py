#!/usr/bin/env python3
"""检查 bag 里 IMU 时间轴的完整性：频率、间隔分布、断点、重复时间戳、静止段 |a|。

历史记录里 kswlt 的桥接会把 IMU 时间轴弄坏（0.5s 卡顿被误判成时钟跳变，
约 100 个样本的时间戳被压成几乎同一时刻）。imu_cam_8 当时判定是干净的，
这个脚本就是把「干净」这件事重新量一遍，并给出量化证据。

用法：check_bag_imu.py <bag> [topic]
"""
import sys

import numpy as np
import rosbag


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag_path = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else '/imu'

    bag = rosbag.Bag(bag_path)
    print(f'bag: {bag_path}')
    print(f'时长 {bag.get_end_time() - bag.get_start_time():.1f} s')
    print('话题清单:')
    for t, ti in sorted(bag.get_type_and_topic_info().topics.items()):
        print(f'  {t:46s} {ti.message_count:7d} 条')
    print()

    ts, accs, gyros = [], [], []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts.append(msg.header.stamp.to_sec())
        accs.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyros.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    bag.close()

    ts = np.array(ts)
    accs, gyros = np.array(accs), np.array(gyros)
    print(f'{topic}: {len(ts)} 条')
    if len(ts) < 2:
        return

    dt = np.diff(ts)
    print(f'  实测频率 {len(ts) / (ts[-1] - ts[0]):.2f} Hz')
    print(f'  dt 中位 {np.median(dt) * 1e3:.3f} ms   均值 {dt.mean() * 1e3:.3f}   '
          f'标准差 {dt.std() * 1e3:.3f}')
    print(f'  dt min {dt.min() * 1e3:.3f} ms   max {dt.max() * 1e3:.3f} ms')
    print(f'  严格单调递增: {bool(np.all(dt > 0))}    重复时间戳: {int(np.sum(dt == 0))} 处')

    big = np.where(dt > 0.05)[0]
    print(f'  间隔 >50ms 的断点: {len(big)} 处')
    for i in big[:10]:
        print(f'    t={ts[i]:.3f}   间隔 {dt[i] * 1e3:.1f} ms')

    tiny = np.where(dt < 1e-4)[0]
    print(f'  间隔 <0.1ms 的挤压点: {len(tiny)} 处')
    for i in tiny[:10]:
        print(f'    t={ts[i]:.6f}   间隔 {dt[i] * 1e6:.1f} us')

    an = np.linalg.norm(accs, axis=1)
    print(f'  |a| 中位 {np.median(an):.4f}   均值 {an.mean():.4f}   std {an.std():.4f}')

    w = int(round(2.0 / np.median(dt)))
    if w > 3 and len(an) > w:
        best = min(range(0, len(an) - w, max(1, w // 2)),
                   key=lambda i: np.std(an[i:i + w]))
        print(f'  最静止 2s 窗口 (t={ts[best]:.2f}):')
        print(f'    |a| 均值 {an[best:best + w].mean():.4f} m/s^2   std {an[best:best + w].std():.5f}')
        print(f'    acc  均值 = {accs[best:best + w].mean(axis=0)}')
        print(f'    gyro 均值 = {gyros[best:best + w].mean(axis=0)} rad/s')
        print(f'    gyro |w| 均值 = {np.linalg.norm(gyros[best:best + w], axis=1).mean():.5f} rad/s')


if __name__ == '__main__':
    main()
