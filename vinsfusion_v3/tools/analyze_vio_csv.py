#!/usr/bin/env python3
"""量 VINS 输出的 vio.csv 的漂移指标。

vio.csv 每行（由 visualization.cpp 写出，11 列）：
  t(ns), x, y, z, qw, qx, qy, qz, vx, vy, vz

重点看三件事：
  1. x/y/z 分开报 —— 合起来算个模长会把「水平跑掉」和「竖直掉下去」混为一谈；
  2. 漂移随时间的增长律 —— x∝t² 指向恒定的残余加速度（零偏/重力方向误差），
     x∝t 指向初速度错，x∝t³ 指向加速度尺度错；
  3. 姿态是否还平 —— 若 q 一直接近单位四元数，说明姿态没崩，
     跑掉的是平移，那问题在「重力抵消得不干净」。

用法：analyze_vio_csv.py <csv> [<csv> ...]
"""
import sys

import numpy as np


def load(p):
    rows = []
    for ln in open(p, errors='replace'):
        ln = ln.strip().rstrip(',')
        if not ln or ln.startswith('#'):
            continue
        parts = ln.split(',')
        if len(parts) < 11:
            continue
        try:
            rows.append([float(x) for x in parts[:11]])
        except ValueError:
            continue
    return np.array(rows)


def growth_law(t, r):
    """把 r(t) 拟合成 a*t^p，返回 p。只在 r 足够大的区段上拟合。"""
    m = (t > 5) & (r > 0.5)
    if m.sum() < 10:
        return None, None
    lt, lr = np.log(t[m]), np.log(r[m])
    p, lna = np.polyfit(lt, lr, 1)
    return p, np.exp(lna)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for path in sys.argv[1:]:
        d = load(path)
        print(f'=== {path} ===')
        if len(d) < 2:
            print(f'  有效行数不足 ({len(d)})，跳过\n')
            continue

        t = d[:, 0] * 1e-9
        t = t - t[0]
        pos = d[:, 1:4]
        quat = d[:, 4:8]
        vel = d[:, 8:11]

        step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
        dt = np.diff(t)
        speed = step / np.maximum(dt, 1e-9)
        n = len(speed)

        print(f'  帧数 {len(d)}   时长 {t[-1]:.1f} s')
        for i, ax in enumerate('xyz'):
            print(f'  终点 {ax} = {pos[-1, i]:+8.2f} m', end='')
        print(f'     水平位移 {np.hypot(*pos[-1, :2]):.2f} m   路径长 {step.sum():.2f} m')

        print(f'  速度(位置差分) 中位 {np.median(speed):.3f}   末段 {np.median(speed[int(n * 0.9):]):.3f} m/s')
        seg = max(1, n // 10)
        vals = [f'{np.median(speed[i * seg:(i + 1) * seg]):.2f}'
                for i in range(10) if i * seg < n]
        print(f'  分段速度中位: ' + ' '.join(vals))

        # 姿态：看四元数的 w 分量（接近 1 = 基本没转）
        qw = np.abs(quat[:, 0])
        ang = 2 * np.degrees(np.arccos(np.clip(qw, 0, 1)))
        print(f'  相对起始的姿态角: 中位 {np.median(ang):.1f}°  最大 {ang.max():.1f}°  '
              f'末值 {ang[-1]:.1f}°')

        r = np.hypot(pos[:, 0], pos[:, 1])
        p, a = growth_law(t, r)
        if p is not None:
            print(f'  水平漂移增长律: r ≈ {a:.4g} * t^{p:.2f}')
            if p > 2.5:
                hint = '加速度尺度错（IMU 标度/时间尺度）'
            elif p > 1.5:
                hint = '恒定残余加速度（加计零偏 或 重力方向没抵消干净）'
            elif p > 0.7:
                hint = '恒定残余速度（初速度估计错）'
            else:
                hint = '不像幂律，另找原因'
            print(f'    → {hint}')
        print()


if __name__ == '__main__':
    main()
