#!/usr/bin/env python3
"""从 vins.log 抽 |pos| 随时间的变化曲线。

为什么单独写这个脚本：vins.log 是 vins_node 一边跑一边追加的，
最后一行经常是写了一半的（实测见到过截断成 "time: 179"），
用 tail -1 抓就会算出 sqrt(179^2)=179 米的假位置。
这里显式丢掉最后一行不完整的；而且只用完整格式的行。

日志行格式（visualization.cpp:173）：
  time: <ts>, t: <x> <y> <z> q: <w> <x> <y> <z>
按空白切分后：$2=时间戳(带逗号) $4$5$6=位置
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/vins_drone/vins.log"
step = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

pat = re.compile(
    r"^time:\s*([0-9.]+),\s*t:\s*([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)"
)

rows = []
with open(path, "r", errors="replace") as f:
    lines = f.readlines()

# 丢掉最后一行 —— 它可能是正在写入的半截行
if lines and not lines[-1].endswith("\n"):
    lines = lines[:-1]

for ln in lines:
    m = pat.match(ln)
    if m:
        ts = float(m.group(1))
        x, y, z = (float(m.group(i)) for i in (2, 3, 4))
        rows.append((ts, (x * x + y * y + z * z) ** 0.5, x, y, z))

if not rows:
    print("   没解析到位姿行")
    sys.exit(1)

t0 = rows[0][0]
print(f"   共 {len(rows)} 条完整位姿行，时长 {rows[-1][0] - t0:.1f}s")
print()
print("      t(s)     |pos|(m)          x          y          z")
nxt = 0.0
for ts, p, x, y, z in rows:
    if ts - t0 >= nxt:
        print(f"   {ts - t0:7.0f}   {p:10.6f}  {x:10.6f} {y:10.6f} {z:10.6f}")
        nxt += step

ts, p, x, y, z = rows[-1]
print()
print(f"   最后一条: t={ts - t0:.1f}s  |pos|={p:.6f} m")
print(f"   分段速度(平均): {p / max(ts - t0, 1e-9) * 60000:.3f} mm/分钟")
