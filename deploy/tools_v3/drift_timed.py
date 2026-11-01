#!/usr/bin/env python3
"""带墙钟时刻的漂移曲线。

为什么需要它：drift_curve.py 只给相对时间，没法跟内核日志、rosout.log 的
绝对时刻对表。vins.log 里的 time: 字段是 epoch（相机时钟），可以直接换算成
HH:MM:SS —— 于是"漂移什么时候跳的"和"相机什么时候掉的"能一眼对上。

同样丢掉最后一行不完整的（曾因此算出 179 米的假位置）。
"""
import datetime
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/vins_drone/vins.log"
step = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
jump = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05  # 相邻采样超过这个增量就标 ★

pat = re.compile(
    r"^time:\s*([0-9.]+),\s*t:\s*([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)"
)

with open(path, "r", errors="replace") as f:
    lines = f.readlines()
if lines and not lines[-1].endswith("\n"):
    lines = lines[:-1]

rows = []
for ln in lines:
    m = pat.match(ln)
    if m:
        ts = float(m.group(1))
        x, y, z = (float(m.group(i)) for i in (2, 3, 4))
        rows.append((ts, (x * x + y * y + z * z) ** 0.5))

if not rows:
    print("   没解析到位姿行")
    sys.exit(1)


def hhmmss(t):
    return datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S")


t0 = rows[0][0]
print(f"   {len(rows)} 条位姿行，{hhmmss(t0)} → {hhmmss(rows[-1][0])}"
      f"（{(rows[-1][0] - t0) / 60:.1f} 分钟）")
print()
print("     墙钟     相对t(s)     |pos|(m)")
nxt = 0.0
prev = 0.0
for ts, p in rows:
    if ts - t0 >= nxt:
        flag = " ★跳变" if abs(p - prev) > jump else ""
        print(f"   {hhmmss(ts)}  {ts - t0:8.0f}  {p:12.6f}{flag}")
        nxt += step
        prev = p

ts, p = rows[-1]
print()
print(f"   最后: {hhmmss(ts)}  相对 {ts - t0:.0f}s  |pos|={p:.6f} m")
