#!/usr/bin/env python3
"""从 vins_node 的运行日志里把 td 的时间轨迹抠出来，看它"什么时候才被允许动"。

为什么需要它：estimator.cpp:1042 的 td 估计是被速度门控的 ——
    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)  problem.SetParameterBlockConstant(para_Td[0]);
也就是说，只要【窗口里最旧那一帧】的速度没超过 0.2 m/s，td 就被钉死。
所以"td 一直不动"不等于"td 收敛了"，可能只是门没开。
配合 estimator.cpp:1029 的外参门（同一族条件），可以解释外参为什么前 25~31 s 冻着。

用法（容器内）: python3 td_timeline.py <log> [<log2> ...] [--from 0 --to 99 --step 2]
每格给出：该时段的 td 末值 + 段内极差（极差 = 0 表示这一整段它根本没动）。
"""
import collections
import re
import sys

args = sys.argv[1:]
lo, hi, step = 0.0, 100.0, 2.0
if '--from' in args:
    lo = float(args[args.index('--from') + 1])
if '--to' in args:
    hi = float(args[args.index('--to') + 1])
if '--step' in args:
    step = float(args[args.index('--step') + 1])
logs = [a for a in args if a.endswith('.log')]

if not logs:
    print(__doc__)
    sys.exit(1)

D = {}
for p in logs:
    rows = []
    for ln in open(p, errors='replace'):
        m = re.search(r'\[(\d+\.\d+)\]: td ([\d.]+)', ln)
        if m:
            rows.append((float(m.group(1)), float(m.group(2))))
    if not rows:
        print(f'跳过 {p}：日志里没有 "td x.xxxxxx" 行')
        continue
    t0 = rows[0][0]
    D[p.rsplit('/', 1)[-1]] = [(t - t0, v) for t, v in rows]

if not D:
    sys.exit(1)

print(f'{"t(s)":>6} |' + ''.join(f' {k[:30]:^34} |' for k in D))
print(f'{"":>6} |' + ''.join(f' {"td 末值":>12}{"段内极差(us)":>20} |' for _ in D))
print('-' * (9 + 37 * len(D)))

s = lo
while s < hi:
    line = f'{s:6.0f} |'
    for lab, rows in D.items():
        vs = [v for t, v in rows if s <= t < s + step]
        if not vs:
            line += ' ' * 36 + '|'
            continue
        line += f' {vs[-1]:12.6f}{(max(vs) - min(vs)) * 1e6:20.1f} |'
    print(line)
    s += step

print()
for lab, rows in D.items():
    frozen = [t for t, v in rows]  # 占位，下面单独算
    # 第一次出现"和初值不同"的时刻 = 门开的时刻
    v0 = rows[0][1]
    moved = [t for t, v in rows if abs(v - v0) > 1e-9]
    if moved:
        print(f'{lab:<34} 初值 {v0:.6f}  首次变动 t≈{moved[0]:.1f} s  末值 {rows[-1][1]:.6f}')
    else:
        print(f'{lab:<34} 初值 {v0:.6f}  全程未动  末值 {rows[-1][1]:.6f}')
