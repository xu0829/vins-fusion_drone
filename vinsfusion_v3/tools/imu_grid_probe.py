#!/usr/bin/env python3
"""判 /imu 到底是【真丢包】还是【飞控本来就在变周期发】。

背景：同一批间隔数据，两种解读都说得通，之前没定论：
  解读A 真丢包  —— 飞控以 4502us 均匀发，中间的消息没送到 → imu_utils 出垃圾
  解读B 变周期  —— 飞控内部有个更细的栅格 G（约1500us），4502≈3G、10504≈7G
                   都是栅格的整数倍，只是相邻两次发送隔了 3 格 / 7 格。
                   这种是"飞控真的发慢了"，不是传输丢的。

两种解读在"最长间隔"上完全一样，但在【全间隔分布】上不一样：
  解读A（栅格=4502）：只该出现 4502 的整数倍附近 —— 4502/9004/13506
  解读B（栅格=1500）：该出现 1500/3000/4500/6000/7500/9000/10500 一整排
所以：看有没有 1500、3000、6000、7500、9000 这些"中间值"。
有 → B，没有 → A。

用法:
    python3 imu_grid_probe.py /tmp/imu_raw.csv
"""
import sys
from collections import Counter

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/imu_raw.csv'
d = np.genfromtxt(path, delimiter=',', names=True)
st = d['sensor_time']
n = len(st)
us = np.round(np.diff(st) * 1e6).astype(int)
span = st[-1] - st[0]

print('样本 %d 条，跨度 %.1f 秒，唯一间隔值 %d 种' % (n, span, len(set(us))))
print()
print('=== 全间隔直方图（前 25 高，不过滤）===')
c = Counter(us)
for v, k in c.most_common(25):
    print('  %8d us  x%-7d %5.2f%%' % (v, k, 100.0 * k / n))
print()

# ---- 栅格搜索：哪个 G 能把最多间隔解释成"整数倍" ----
print('=== 最佳栅格搜索（谁当基准，整数倍命中率最高）===')
res = []
for G in np.arange(1200.0, 5200.0, 0.5):
    r = us / G
    hit = np.mean(np.abs(r - np.round(r)) < 0.004)
    res.append((hit, G))
res.sort(reverse=True)
seen = []
for hit, G in res:
    if any(abs(G - s) / s < 0.02 for s in seen):
        continue
    seen.append(G)
    print('  G = %7.1f us (%6.1f Hz)  ->  整数倍命中 %.2f%%' % (G, 1e6 / G, 100 * hit))
    if len(seen) >= 5:
        break
print()

# ---- 决定性判据：有没有"中间值" ----
print('=== 决定性判据：基准 4502 的整数倍之间，有没有东西 ===')
base = c.most_common(1)[0][0]
print('  最常见间隔（基准）= %d us' % base)
for k in range(1, 5):
    lo, hi = base * k * 0.80, base * k * 1.20
    if k > 1:
        lo = max(lo, base * (k - 1) * 1.20)
    m = (us >= lo) & (us <= hi)
    print('  第%d倍附近 [%d,%d] us: %d 条' % (k, lo, hi, m.sum()))
print()
# 具体看 1500 那一排存不存在
print('  1500 栅格的各倍点（±1.5%% 窗内条数）：')
for G, lbl in ((1500.7, 'A: 若这是栅格'), ):
    for k in range(1, 9):
        tgt = G * k
        m = np.abs(us - tgt) <= tgt * 0.015
        print('    %2d x %.1f = %7.1f us : %6d 条  %s'
              % (k, G, tgt, m.sum(), '#' * min(40, m.sum() // 30)))
print()
print('  4502 栅格的各倍点（±1.5%% 窗内条数）：')
for k in range(1, 5):
    tgt = 4502.0 * k
    m = np.abs(us - tgt) <= tgt * 0.015
    print('    %2d x 4502.0 = %7.1f us : %6d 条  %s'
          % (k, tgt, m.sum(), '#' * min(40, m.sum() // 30)))
