#!/usr/bin/env bash
# 在容器内用现成的 ROS1 bag 回放，跑 vins_node 做端到端验证。
#
# 用法（容器内，已 source /opt/ros/noetic/setup.bash）：
#   bash tools/run_vins_replay.sh <主配置路径> <bag路径> [速率] [时长秒]
#
# 例：
#   bash tools/run_vins_replay.sh config/d430_px4/d430_px4_stereo_imu_config.yaml \
#                                /root/calib_data/imu_cam_8.bag 1.0
#   bash tools/run_vins_replay.sh <配置> <bag> 1.0 45     # 只回放前 45 秒（做确定性测试用）
#
# 输出目录取自配置里的 output_path（不是写死的），并会：
#   1. 先 mkdir -p —— VINS 用 std::ofstream 写 vio.csv，目录不存在会【静默】失败；
#   2. 跑完把 vio.csv 另存为 vio_<时间戳>.csv —— VINS 用 ios::out 打开，每跑一趟都会覆盖。
# 这两条都是踩过的坑，别再退化回去。
set -u

# vins_node 是 catkin 工作空间里的包，必须 source 过 devel/setup.bash 才能被 rosrun 找到
if [[ -f /root/catkin_ws/devel/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /root/catkin_ws/devel/setup.bash
fi
if ! rospack find vins >/dev/null 2>&1; then
  echo "找不到 'vins' 包。请先 source /root/catkin_ws/devel/setup.bash（或确认已编译）"
  exit 1
fi

CONFIG="${1:?需要主配置路径}"
BAG="${2:?需要 bag 路径}"
RATE="${3:-1.0}"
DUR="${4:-0}"   # >0 则只回放前 DUR 秒（确定性测试用，省时间）

# 输出目录 = 配置里的 output_path
OUT_DIR=$(grep -m1 '^output_path:' "$CONFIG" \
          | sed -e 's/^output_path:[[:space:]]*//' -e 's/^"//' -e 's/"[[:space:]]*$//')
if [[ -z "$OUT_DIR" ]]; then
  echo "配置 $CONFIG 里读不到 output_path"
  exit 1
fi
mkdir -p "$OUT_DIR"

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$OUT_DIR/replay_${STAMP}.log"

echo "配置: $CONFIG"
echo "bag : $BAG  (速率 $RATE)"
echo "输出: $OUT_DIR"
echo "日志: $LOG"
echo

# 把跑的是哪份配置写进日志头，否则事后无从分辨（踩过：五份日志认不出谁是谁）
{
  echo "########## CONFIG: $CONFIG"
  echo "########## BAG: $BAG  RATE: $RATE  DURATION: ${DUR}s (0=全片)"
  echo "########## START: $(date -Iseconds)"
  echo
} > "$LOG"

# --- 起 roscore（若已有则复用）---
if ! rostopic list >/dev/null 2>&1; then
  echo "[1/4] 启动 roscore"
  setsid nohup roscore > /tmp/roscore.log 2>&1 < /dev/null &
  for _ in $(seq 1 30); do
    rostopic list >/dev/null 2>&1 && break
    sleep 0.5
  done
  rostopic list >/dev/null 2>&1 || { echo "roscore 起不来，看 /tmp/roscore.log"; exit 1; }
else
  echo "[1/4] roscore 已在运行"
fi

# --- 起 vins_node ---
echo "[2/4] 启动 vins_node"
setsid nohup rosrun vins vins_node "$CONFIG" >> "$LOG" 2>&1 < /dev/null &
VINS_PID=$!
sleep 4
if ! kill -0 "$VINS_PID" 2>/dev/null; then
  echo "vins_node 立即退出了，日志尾部："
  tail -20 "$LOG"
  exit 1
fi
echo "      pid=$VINS_PID"

# --- 回放 ---
if (( DUR > 0 )); then
  echo "[3/4] 回放 bag（前 ${DUR}s，速率 $RATE）"
  rosbag play -r "$RATE" --duration "$DUR" "$BAG" > /tmp/bagplay.log 2>&1
else
  echo "[3/4] 回放 bag（约 99s，速率 $RATE）"
  rosbag play -r "$RATE" "$BAG" > /tmp/bagplay.log 2>&1
fi
echo "      回放结束"

# --- 收尾 ---
echo "[4/4] 停止 vins_node，等它把轨迹写完"
sleep 3
kill -INT "$VINS_PID" 2>/dev/null
sleep 3
kill -9 "$VINS_PID" 2>/dev/null

# --- 另存本次轨迹 ---
SAVED=""
if [[ -f "$OUT_DIR/vio.csv" ]]; then
  SAVED="$OUT_DIR/vio_${STAMP}.csv"
  cp "$OUT_DIR/vio.csv" "$SAVED"
fi

echo
echo "===== vins_node 自己汇报的量 ====="
grep -aE "fix extrinsic|Optimize extrinsic|gyroscope bias|Initialization finish|not enough|result path" "$LOG" | head -10
echo
echo "===== 日志尾部 ====="
tail -6 "$LOG"
echo
echo "===== 轨迹产物 ====="
if [[ -n "$SAVED" ]]; then
  echo "$SAVED  ($(wc -l < "$SAVED") 行)"
else
  echo "(未生成 vio.csv)"
fi
