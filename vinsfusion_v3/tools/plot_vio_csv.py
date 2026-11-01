#!/usr/bin/env python3
"""把 vio.csv 画成图：x/y/z 随时间曲线 + 轨迹俯视图。

漂移这件事，看曲线比看 3D 视图还直观 —— 3D 里"飞出去"到底飞了多远要靠感觉，
曲线上是多少米一目了然，而且多趟叠在一起对比很方便。

用法：plot_vio_csv.py <输出png> <csv> [<csv> ...]
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


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


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    out_png, paths = sys.argv[1], sys.argv[2:]

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2))

    for p in paths:
        d = load(p)
        if len(d) < 2:
            print(f'{p}: 有效行数不足，跳过')
            continue
        t = d[:, 0] * 1e-9
        t = t - t[0]
        pos = d[:, 1:4]
        name = p.rsplit('/', 1)[-1]

        # 左：x/y/z 随时间
        for i, (ax, lab) in enumerate(zip('xyz', ('红 x', '绿 y', '蓝 z'))):
            axes[0].plot(t, pos[:, i], lw=1.2, label=f'{name} {ax}')
        # 中：水平漂移模长
        axes[1].plot(t, np.hypot(pos[:, 0], pos[:, 1]), lw=1.2, label=name)
        # 右：俯视轨迹
        axes[2].plot(pos[:, 0], pos[:, 1], lw=1.2, label=name)
        axes[2].plot(pos[0, 0], pos[0, 1], 'go', ms=7)
        axes[2].plot(pos[-1, 0], pos[-1, 1], 'rx', ms=9, mew=2)

    axes[0].set_xlabel('t (s)')
    axes[0].set_ylabel('position (m)')
    axes[0].set_title('x / y / z 随时间')
    axes[0].legend(fontsize=7, ncol=2)
    axes[0].grid(alpha=.3)

    axes[1].set_xlabel('t (s)')
    axes[1].set_ylabel('sqrt(x^2+y^2)  (m)')
    axes[1].set_title('水平漂移量')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=.3)

    axes[2].set_xlabel('x (m)')
    axes[2].set_ylabel('y (m)')
    axes[2].set_title('轨迹俯视图（绿=起点，红=终点）')
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=.3)
    axes[2].axis('equal')

    fig.suptitle('VINS-Fusion 轨迹：漂移一眼可见', fontsize=13)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print(f'已写出 {out_png}')


if __name__ == '__main__':
    main()
