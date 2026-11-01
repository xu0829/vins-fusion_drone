#!/usr/bin/env bash
# 同一份配置重复跑 N 趟，检查跑批是否可重复。
#
# 为什么需要这个：VINS-Fusion 的 multiple_thread: 1 会让特征跟踪跑在线程池里，
# 线程调度抖动 -> 关键帧选取不同 -> 初始化窗口内容不同 -> 陀螺零偏初值不同。
# 实测同一份配置同一份 bag，两趟的终点水平位移差了 1.8 倍（82 m vs 146 m），
# 于是任何 A/B 对比都淹没在这个抖动里。要做对比，先把这个消掉。
#
# 用法（容器内）: repeat_runs.sh <配置> <bag> <趟数> <标签>
# 产物: <output_path>/<标签>_run<i>.csv             每趟的轨迹
#       <output_path>/<标签>_run<i>_extrinsic.yaml 每趟的外参（整体拷贝）
#
# 坑（踩过，别再退化）：extrinsic_parameter.csv 由 visualization.cpp:90 用
# cv::FileStorage(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE) 写 ——
# 【整体重写】而非追加，下一趟 vins_node 一启动就把它截断重写。
# 所以不能靠"文件尺寸增量"取新记录，必须每趟结束时整体拷走。
set -u

CONFIG="${1:?需要配置路径}"
BAG="${2:?需要 bag 路径}"
N="${3:-2}"
TAG="${4:-run}"

OUT_DIR=$(grep -m1 '^output_path:' "$CONFIG" \
          | sed -e 's/^output_path:[[:space:]]*//' -e 's/^"//' -e 's/"[[:space:]]*$//')
if [[ -z "$OUT_DIR" ]]; then
  echo "配置 $CONFIG 里读不到 output_path"
  exit 1
fi
mkdir -p "$OUT_DIR"
EX="$OUT_DIR/extrinsic_parameter.csv"

echo "配置 : $CONFIG"
echo "输出 : $OUT_DIR"
echo "趟数 : $N   标签: $TAG"
echo

SAVED=()
for i in $(seq 1 "$N"); do
  echo "############ [$TAG] 第 $i/$N 趟 ############"

  bash /root/vinsfusion/tools/run_vins_replay.sh "$CONFIG" "$BAG" 1.0 2>&1 \
    | grep -aE "gyroscope bias|Initialization finish|td [0-9]|fix extrinsic|Optimize extrinsic|轨迹产物|vio_"

  LATEST=$(ls -t "$OUT_DIR"/vio_*.csv 2>/dev/null | head -1)
  if [[ -n "$LATEST" ]]; then
    DST="$OUT_DIR/${TAG}_run${i}.csv"
    cp "$LATEST" "$DST"
    SAVED+=("$DST")
    echo "  轨迹 -> $(basename "$DST")"
  else
    echo "  !! 没找到本趟轨迹"
  fi

  if [[ -f "$EX" ]]; then
    DEX="$OUT_DIR/${TAG}_run${i}_extrinsic.yaml"
    cp "$EX" "$DEX"
    echo "  外参 -> $(basename "$DEX")  ($(stat -c%s "$DEX") 字节)"
    python3 /root/vinsfusion/tools/read_extrinsic.py "$DEX" 2>/dev/null | sed 's/^/    /'
  else
    echo "  (本趟没写出 extrinsic_parameter.csv —— 该配置大概没开 estimate_extrinsic)"
  fi
  echo
done

echo "############ 汇总：$N 趟的轨迹对比 ############"
if (( ${#SAVED[@]} > 0 )); then
  python3 /root/vinsfusion/tools/analyze_vio_csv.py "${SAVED[@]}"
fi
