#!/usr/bin/env bash
# 确定性测试：同一配置、限定时长重复跑，逐行比对轨迹是否一致。
#
# 判据是「数值是否逐位一致」，不靠漂移大小 —— 所以截短到 45 秒足够暴露抖动，
# 比跑满 99 秒省一半时间。（抖动是从第一行就开始累积的：实测两趟在同一行号上
# 第 1 行差 0.00017 m、第 1000 行差 0.029 m、终值差 21 m。）
#
# 用法（容器内，已 source /opt/ros/noetic/setup.bash）：
#   bash tools/determinism_test.sh <配置> <bag> <趟数> [时长秒] [速率]
set -u

CONFIG="${1:?需要配置}"
BAG="${2:?需要 bag}"
N="${3:-2}"
DUR="${4:-45}"
RATE="${5:-1.0}"
TAG="det"

OUT_DIR=$(grep -m1 '^output_path:' "$CONFIG" \
          | sed -e 's/^output_path:[[:space:]]*//' -e 's/^"//' -e 's/"[[:space:]]*$//')
mkdir -p "$OUT_DIR"

echo "配置: $CONFIG"
echo "输出: $OUT_DIR"
echo "趟数: $N   时长: ${DUR}s   速率: $RATE"
echo

SAVED=()
for i in $(seq 1 "$N"); do
  echo "############ 第 $i/$N 趟 ############"
  bash /root/vinsfusion/tools/run_vins_replay.sh "$CONFIG" "$BAG" "$RATE" "$DUR" 2>&1 \
    | grep -aE "gyroscope bias|Initialization finish|vio_|未生成"
  LATEST=$(ls -t "$OUT_DIR"/vio_*.csv 2>/dev/null | head -1)
  if [[ -n "$LATEST" ]]; then
    DST="$OUT_DIR/${TAG}_run${i}.csv"
    cp "$LATEST" "$DST"
    SAVED+=("$DST")
    echo "  -> $(basename "$DST")  ($(wc -l < "$DST") 行)"
  else
    echo "  !! 没找到轨迹"
  fi
  echo
done

echo "############ 逐行比对 ############"
if (( ${#SAVED[@]} < 2 )); then
  echo "趟数不足 2，无法比对"
  exit 1
fi

python3 - "${SAVED[@]}" <<'PY'
import sys

import numpy as np


def load(p):
    rows = []
    for ln in open(p, errors='replace'):
        ln = ln.strip().rstrip(',')
        if not ln or ln.startswith('#'):
            continue
        q = ln.split(',')
        if len(q) < 11:
            continue
        try:
            rows.append([float(x) for x in q[:11]])
        except ValueError:
            pass
    return np.array(rows)


names = sys.argv[1:]
data = [load(p) for p in names]

print('各趟行数: ' + '  '.join(f'{n.rsplit("/", 1)[-1]}={len(d)}' for n, d in zip(names, data)))
print()
base = data[0]
all_same = True
for n, d in zip(names[1:], data[1:]):
    if len(d) != len(base):
        print(f'{n} 行数与首趟不同（{len(d)} vs {len(base)}）—— 已经不一致')
        all_same = False
        continue
    diff = np.abs(d - base)
    # 时间列（纳秒）单独看，避免和位置混在一起
    dt_ns = diff[:, 0].max()
    dpos = diff[:, 1:4]
    print(f'{n.rsplit("/",1)[-1]}:')
    print(f'   时间列最大差 {dt_ns:.0f} ns')
    print(f'   位置最大差 {dpos.max():.6e} m   中位 {np.median(dpos):.3e} m')
    for k in (0, 99, 499, len(base) - 1):
        if k < len(base):
            print(f'     第 {k + 1:5d} 行: {dpos[k].max():.6e} m')
    if dpos.max() == 0 and dt_ns == 0:
        print('   -> 逐位一致')
    else:
        print('   -> 不一致')
        all_same = False

print()
print('结论:', '可重复 —— 逐位一致' if all_same else '仍不可重复，抖动源还没消掉')
PY
