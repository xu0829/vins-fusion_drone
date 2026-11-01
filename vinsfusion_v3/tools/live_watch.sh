#!/usr/bin/env bash
# 直播看漂移：回放 bag 跑 VINS，同时开 foxglove_bridge，并把
#   /vins_estimator/extrinsic   （外参随时间的实际取值）
#   /vins_estimator/odometry    （轨迹/速度）
# 记成 bag，跑完可以离线复盘。
#
# 和 start_viz.sh 的区别：这个不按回车、全自动，适合后台跑；
# 而且多录了 extrinsic 话题 —— 这是回答"外参每次是否收敛到同一个答案"的关键，
# 因为 extrinsic_parameter.csv 是【整体重写】的，只能看到最后一帧的值。
#
# 用法（容器内）: bash tools/live_watch.sh <配置> <bag> [速率] [标签]
#
# 标签给了就把本趟轨迹另存为 <标签>_run<i>.csv + <标签>_extrinsic_trace.bag，
# 方便重复跑之后并排比对（否则 vio.csv 每趟被覆盖）。
set -u
PORT=${PORT:-8767}

if [[ -f /root/catkin_ws/devel/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /root/catkin_ws/devel/setup.bash
fi

CONFIG="${1:?需要主配置路径}"
BAG="${2:?需要 bag 路径}"
RATE="${3:-1.0}"
TAG="${4:-}"

OUT_DIR=$(grep -m1 '^output_path:' "$CONFIG" \
          | sed -e 's/^output_path:[[:space:]]*//' -e 's/^"//' -e 's/"[[:space:]]*$//')
mkdir -p "$OUT_DIR"
if [[ -n "$TAG" ]]; then
  STEM="$OUT_DIR/${TAG}_$(date +%m%d_%H%M%S)"
else
  STEM="$OUT_DIR/extrinsic_trace_$(date +%Y%m%d_%H%M%S)"
fi
REC="${STEM}.bag"

# --- roscore ---
if ! rostopic list >/dev/null 2>&1; then
  echo "[1/5] 启动 roscore"
  setsid nohup roscore > /tmp/roscore.log 2>&1 < /dev/null &
  for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
else
  echo "[1/5] roscore 已在运行"
fi

# --- foxglove_bridge ---
if timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
  echo "[2/5] foxglove_bridge 已在 $PORT"
else
  echo "[2/5] 启动 foxglove_bridge（端口 $PORT）"
  setsid nohup rosrun foxglove_bridge foxglove_bridge \
    _port:="$PORT" _address:=0.0.0.0 > /tmp/foxglove_bridge.log 2>&1 < /dev/null &
  for _ in $(seq 1 20); do
    timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null && break; sleep 0.5
  done
  timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null \
    && echo "      桥已就绪" || { echo "      桥没起来:"; tail -10 /tmp/foxglove_bridge.log; }
fi

# --- vins_node ---
echo "[3/5] 启动 vins_node（$CONFIG）"
setsid nohup rosrun vins vins_node "$CONFIG" > "$OUT_DIR/live_$(date +%H%M%S).log" 2>&1 < /dev/null &
VINS_PID=$!
sleep 4
kill -0 "$VINS_PID" 2>/dev/null || { echo "vins_node 立即退出"; exit 1; }

# --- 录制 ---
# point_cloud 一起录：它的点数就是当前被三角化/参与优化的特征数。
# t≈31 s 跑飞时如果点数塌了，说明是视觉约束先垮（跟丢），IMU 独撑才导致速度失控；
# 如果点数一直很多，那问题就在 IMU 侧。这条能直接分开两种病因。
echo "[4/5] 开始录制外参/轨迹/特征点数 -> $REC"
setsid nohup rosbag record -O "$REC" \
  /vins_estimator/extrinsic /vins_estimator/odometry /vins_estimator/point_cloud \
  > /tmp/bagrecord.log 2>&1 < /dev/null &
REC_PID=$!
sleep 1

# --- 回放 ---
echo "[5/5] 回放 bag（$BAG，速率 $RATE）"
echo "      Foxglove 连 ws://127.0.0.1:$PORT"
rosbag play -r "$RATE" "$BAG"

echo "      回放结束，等 3 秒收尾"
sleep 3
kill -INT "$REC_PID" 2>/dev/null; sleep 2
kill -INT "$VINS_PID" 2>/dev/null; sleep 2
kill -9 "$VINS_PID" 2>/dev/null

echo
echo "录制文件: $REC"
ls -la "$REC" 2>/dev/null || echo "(没录到)"

# vio.csv 是 ios::out 打开的，每趟覆盖，所以跑完立刻另存
if [[ -n "$TAG" && -f "$OUT_DIR/vio.csv" ]]; then
  cp "$OUT_DIR/vio.csv" "${STEM}.csv"
  echo "轨迹文件: ${STEM}.csv ($(wc -l < "${STEM}.csv") 行)"
fi
echo "收工: pkill -f foxglove_bridge; pkill -f vins_node"
