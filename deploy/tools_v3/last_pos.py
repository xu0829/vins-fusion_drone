#!/usr/bin/env python3
"""打印 vins.log 里最后一条**完整**位姿行的 |pos|（米），就一个数。

给值守脚本当探针用。丢掉最后一行不完整的 —— 老坑：vins.log 是边跑边追加的，
tail -1 抓到半截行会算出假位置（见过截断成 "time: 179" 于是报 179 米）。

健康参考：静止时应该是 **毫米级**（实测 0.0057 m）。
越过 0.5 米就是发散，不用等它飞到几公里。
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/vins_drone/vins.log"
pat = re.compile(
    r"^time:\s*([0-9.]+),\s*t:\s*([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)"
)

try:
    with open(path, "r", errors="replace") as f:
        lines = f.readlines()
except OSError:
    print("nan")
    sys.exit(0)

if lines and not lines[-1].endswith("\n"):
    lines = lines[:-1]

for ln in reversed(lines):
    m = pat.match(ln)
    if m:
        x, y, z = (float(m.group(i)) for i in (2, 3, 4))
        print(f"{(x * x + y * y + z * z) ** 0.5:.4f}")
        sys.exit(0)
print("nan")
