#!/bin/bash
# =============================================================================
# record_pitch_test.sh —— 快速俯仰 Z 台阶试验 · 自动录包
#
# 在香橙派容器 vins_pi 内运行。只订阅话题、只读日志，不改 VINS/PX4 任何参数。
#
# 用法（容器内）：
#   bash /root/xu_drone/live/record_pitch_test.sh --check     # 只预检，不录
#   bash /root/xu_drone/live/record_pitch_test.sh             # 正式录制
#   bash /root/xu_drone/live/record_pitch_test.sh --name pitch_01
#
# 参数：
#   --check         只做预检，不录制
#   --name NAME     包名（默认 pitch_test_<本地时间>）
#   --no-images     不录双目原图（图像 header.stamp 仍在 /camera/infraN/metadata；抽查前 8 帧完全一致）
#   --cycles N      重复次数，默认 5
#   --move S        每次"快动"窗口秒数，默认 2
#   --hold S        每次保持秒数，默认 5
#
# 输出（容器内）：
#   /root/calib_data/vins_output/pitch_bags/<name>/
#       <name>.bag              ROS1 bag（原始图像，未压缩）
#       meta/                   配置、代码、日志切片、阶段时间线、bag info
# 香橙派主机同一目录：
#   /home/orangepi/vins_output/pitch_bags/<name>/
#
# 重要：PX4 ULog 不走 ROS，本脚本无法代录。测前确认飞控 SD 日志已开，
#       测后停止栈、用 QGC 下载覆盖本时段的 .ulg（见 README）。
# =============================================================================
set -u

export TZ="${TZ:-Asia/Shanghai}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

# 必须在 source 之前给 ROS_MASTER_URI/ROS_HOSTNAME 赋值，否则 catkin 的
# profile.d/10.roslaunch.sh 在 set -u 下会 unbound variable 退出。
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f /root/catkin_ws/devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source /root/catkin_ws/devel/setup.bash
fi

# ---------------- 默认参数 ----------------
CHECK_ONLY=0
NAME_ARG=""
WITH_IMAGES=1
CYCLES="${CYCLES:-5}"
MOVE_S="${MOVE_S:-2}"
HOLD_S="${HOLD_S:-5}"
STATIC_HEAD_S="${STATIC_HEAD_S:-10}"
STATIC_TAIL_S="${STATIC_TAIL_S:-10}"
BAG_ROOT="${BAG_ROOT:-/root/calib_data/vins_output/pitch_bags}"
MIN_FREE_GB="${MIN_FREE_GB:-8}"

usage() {
  sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    --check|-c) CHECK_ONLY=1; shift ;;
    --name)
      [ $# -ge 2 ] || { echo "[错误] --name 需要参数"; exit 2; }
      NAME_ARG="$2"; shift 2 ;;
    --no-images) WITH_IMAGES=0; shift ;;
    --cycles)
      [ $# -ge 2 ] || { echo "[错误] --cycles 需要参数"; exit 2; }
      CYCLES="$2"; shift 2 ;;
    --move)
      [ $# -ge 2 ] || { echo "[错误] --move 需要参数"; exit 2; }
      MOVE_S="$2"; shift 2 ;;
    --hold)
      [ $# -ge 2 ] || { echo "[错误] --hold 需要参数"; exit 2; }
      HOLD_S="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[错误] 未知参数：$1"; usage; exit 2 ;;
  esac
done

is_uint() { case "$1" in ''|*[!0-9]*) return 1 ;; *) return 0 ;; esac; }
for v in "$CYCLES" "$MOVE_S" "$HOLD_S" "$STATIC_HEAD_S" "$STATIC_TAIL_S" "$MIN_FREE_GB"; do
  is_uint "$v" || { echo "[错误] 参数必须是正整数：$v"; exit 2; }
  [ "$v" -ge 1 ] || { echo "[错误] 参数必须 ≥1：$v"; exit 2; }
done

say()  { printf '%s\n' "$*"; }
warn() { printf '[警告] %s\n' "$*" >&2; }
die()  { printf '[错误] %s\n' "$*" >&2; exit 1; }

# ---------------- 固定路径 ----------------
LOG_DIR=/tmp/vins_pi
VINS_LOG="$LOG_DIR/vins.log"
CAM_LOG="$LOG_DIR/camera.log"
DIAG_LOG="$LOG_DIR/diag.log"
IMU_LOG=/tmp/imu_node_recover.log
IMU_SUPER="$LOG_DIR/imu_super.log"
# px4_imu_node.py 在 IMU_CSV 环境变量指向的文件里逐样本记录
#   sensor_time（PX4 HIGHRES_IMU.time_usec*1e-6）, arrival_time（宿主 time.time()）, stamp（映射后 ROS 时间）, scale_ppm
# 这是检验 HIGHRES_IMU.time_usec → /imu.header.stamp 的唯一原始证据。
# IMU CSV：节点默认写 /tmp；若已按建议改到 /dev/shm，自动取较新的那个
if [ -f /dev/shm/imu_raw_recover.csv ] && \
   { [ ! -f /tmp/imu_raw_recover.csv ] || [ /dev/shm/imu_raw_recover.csv -nt /tmp/imu_raw_recover.csv ]; }; then
  IMU_CSV=/dev/shm/imu_raw_recover.csv
else
  IMU_CSV=/tmp/imu_raw_recover.csv
fi
VISION_CSV="$LOG_DIR/vision_uplink.csv"
PX4TELEM_CSV="$LOG_DIR/px4_telemetry.csv"
PARAM_CSV="$LOG_DIR/px4_params.csv"
LIVE=/root/xu_drone/live

topic_listed() { rostopic list 2>/dev/null | grep -qxF "$1"; }
has_data()     { timeout 5 rostopic echo -n 1 "$1" >/dev/null 2>&1; }
rate_of()      { timeout 4 rostopic hz "$1" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}'; }
stat_size()    { if [ -f "$1" ]; then stat -c %s "$1" 2>/dev/null || echo 0; else echo 0; fi; }
mark() {
  local label="$1" note="$2"
  local t iso
  t=$(date +%s.%N)
  iso=$(date -Is)
  note=${note//\"/\"\"}
  printf '%s,%.6f,%s,"%s"\n' "$label" "$t" "$iso" "$note" >> "$STAGE_LOG"
}

# ---------------- 预检 ----------------
echo "================================================================"
echo " 快速俯仰 Z 台阶试验 · 录包"
echo "================================================================"
echo " 主机: $(hostname)   时间: $(date -Is)   TZ=$TZ"
echo

say "[1/6] ROS 与核心话题"
if ! timeout 5 rostopic list >/dev/null 2>&1; then
  die "roscore 不可达（$ROS_MASTER_URI）—— 先在容器里把 VINS 栈起起来"
fi
say "  ✓ roscore 可达（$ROS_MASTER_URI）"

CORE_TOPICS=(
  /imu
  /camera/infra1/image_rect_raw
  /camera/infra2/image_rect_raw
  /vins_estimator/odometry
  /vins_estimator/point_cloud
)
for t in "${CORE_TOPICS[@]}"; do
  topic_listed "$t" || die "核心话题不存在：$t —— VINS/相机/IMU 桥没在跑？"
  has_data "$t"    || die "核心话题没有数据：$t —— 检查 VINS 日志 $VINS_LOG"
  say "  ✓ $t"
done

say
say "[2/6] 频率与实时性"
IMU_HZ=$(rate_of /imu || true)
IMG1_HZ=$(rate_of /camera/infra1/image_rect_raw || true)
IMG2_HZ=$(rate_of /camera/infra2/image_rect_raw || true)
ODOM_HZ=$(rate_of /vins_estimator/odometry || true)
PC_HZ=$(rate_of /vins_estimator/point_cloud || true)
printf '  %-42s %s Hz\n' /imu "${IMU_HZ:--}"
printf '  %-42s %s Hz\n' /camera/infra1/image_rect_raw "${IMG1_HZ:--}"
printf '  %-42s %s Hz\n' /camera/infra2/image_rect_raw "${IMG2_HZ:--}"
printf '  %-42s %s Hz\n' /vins_estimator/odometry "${ODOM_HZ:--}"
printf '  %-42s %s Hz\n' /vins_estimator/point_cloud "${PC_HZ:--}"
if [ "$WITH_IMAGES" -eq 0 ]; then
  say "  不录原图；图像 header.stamp 从 /camera/infraN/metadata 取（抽查前 8 帧与 image 完全一致）"
fi
UPLINK_MODE=$(cat "$LIVE/vision_uplink.mode" 2>/dev/null | tr -d '[:space:]')
say "  视觉上行模式：${UPLINK_MODE:-?}（0=关，2/3=发）"
if [ -n "$UPLINK_MODE" ] && [ "$UPLINK_MODE" != "0" ]; then
  warn "视觉上行开着：会向飞控请求遥测流；实测会把 /imu 从 196 Hz 拉到约 150 Hz。做 VINS/时间基准阶段建议先 echo 0 > $LIVE/vision_uplink.mode（1 秒内生效，不用重启）"
fi
hz_ge() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a >= b) }'; }
if [ -n "$IMU_HZ" ] && ! hz_ge "$IMU_HZ" 180; then
  warn "/imu 频率低于 180 Hz（标称约 196 Hz，低于此值说明有丢样）"
fi
if [ -n "$IMG1_HZ" ] && ! hz_ge "$IMG1_HZ" 10; then warn "infra1 频率低于 10 Hz"; fi
if [ -n "$ODOM_HZ" ] && ! hz_ge "$ODOM_HZ" 5; then warn "VINS odometry 频率低于 5 Hz"; fi

# ★ 必查：VINS 是不是在实时运行。
# odom 的 header.stamp 若落后墙钟 >0.5s，vision_uplink 会主动停发（MAX_LATENCY_S），
# PX4 端就收不到视觉；这种包录下来 VINS 和 PX4 没法对齐，等于白录。
lag_of() {
  timeout 3 rostopic delay "$1" 2>/dev/null \
    | awk '/average delay/ {v = $3} END {if (v != "") print v}'
}
LAST_LAG=""
wait_realtime() {
  local topic="$1" label="$2" tries="${3:-6}" i lag
  for i in $(seq 1 "$tries"); do
    lag=$(lag_of "$topic")
    if [ -n "$lag" ] && awk -v a="$lag" 'BEGIN { a = (a < 0 ? -a : a); exit !(a <= 0.5) }'; then
      LAST_LAG="$lag"
      say "  ✓ $label 实时（|延迟| ${lag}s ≤ 0.5s）"
      return 0
    fi
    LAST_LAG="$lag"
    if [ "$i" -lt "$tries" ]; then
      warn "$label 落后 ${lag:-?}s（>0.5s），等 3s 再看（$i/$tries）"
      sleep 3
    fi
  done
  warn "$label 仍不是实时（最后 ${lag:-?}s）"
  return 1
}
ODOM_LAG=""
PC_LAG=""
wait_realtime /vins_estimator/odometry "VINS odometry" 6 && ODOM_LAG="$LAST_LAG" \
  || die "VINS 落后实时超过 0.5s。此时录的 odom 时间戳整体滞后、vision_uplink 还会停发。
     先把栈降载再重启：相机 infra_fps 改 15 / 关 Foxglove / 关不必要的诊断节点。"
if [ -n "$ODOM_LAG" ] && awk -v a="$ODOM_LAG" 'BEGIN { a = (a < 0 ? -a : a); exit !(a > 0.3) }'; then
  warn "odom 延迟 ${ODOM_LAG}s 已接近 uplink 的 0.5s 门限，测试中可能偶发停发；建议先关 Foxglove/降载"
fi
wait_realtime /vins_estimator/point_cloud "VINS point_cloud" 4 && PC_LAG="$LAST_LAG" \
  || warn "point_cloud 也滞后；若 odom 已实时可继续，但特征数量会晚到"

if ps -eo args 2>/dev/null | grep -q '[f]oxglove_nodelet_manager'; then
  warn "Foxglove 还开着（实测约 12% CPU）—— 测试前建议关掉，给 VINS 留余量"
fi
say "  负载：$(cat /proc/loadavg 2>/dev/null || echo '?')（核数 $(nproc 2>/dev/null || echo '?')）"
say "  温度：$(awk '{printf "%.0f°C ", $1/1000}' /sys/class/thermal/thermal_zone*/temp 2>/dev/null || echo '?')"

say
say "[3/6] VINS 配置 / 初始化 / 上锁状态"
VINS_CMD=$(ps -eo args | grep -m1 '[v]ins_node' || true)
VINS_CFG=""
if [ -n "$VINS_CMD" ]; then
  VINS_CFG=$(printf '%s\n' "$VINS_CMD" | awk '{print $NF}' | grep -E '\.yaml$' || true)
fi
if [ -n "$VINS_CFG" ] && [ -f "$VINS_CFG" ]; then
  say "  配置：$VINS_CFG"
  grep -E '^[[:space:]]*(acc_n|gyr_n|acc_w|gyr_w|estimate_td|estimate_extrinsic|max_cnt|multiple_thread|td):' "$VINS_CFG" \
    | sed 's/^/    /'
else
  warn "没找到 vins_node 命令行/配置文件"
fi
INIT_N=$(grep -a -c 'Initialization finish' "$VINS_LOG" 2>/dev/null || true)
say "  vins.log 初始化完成次数：${INIT_N:-0}"
if [ "${INIT_N:-0}" -eq 0 ]; then
  warn "vins.log 里没有 'Initialization finish' —— 若栈刚重启，先确认 odometry 已稳定"
fi

# vision_uplink 每秒往 /rosout 发一行状态（含 mode= 与 已上锁/已解锁）。
# 不能用 rostopic echo | grep：echo 会把中文转成 \uXXXX，长行还会折行。
status_line_scan() {
  timeout 12 python3 - <<'PY' 2>/dev/null || true
import sys
import rospy
from rosgraph_msgs.msg import Log

rospy.init_node("pitch_test_preflight", anonymous=True, disable_signals=True)
found = []

def cb(m):
    if "[vision]" in m.msg and "mode=" in m.msg:
        found.append(m.msg)

rospy.Subscriber("/rosout", Log, cb)
deadline = rospy.get_time() + 7.0
while not rospy.is_shutdown() and rospy.get_time() < deadline:
    rospy.sleep(0.1)
if found:
    sys.stdout.write(found[-1] + "\n")
PY
}

STATUS_LINE=$(status_line_scan || true)
[ -n "$STATUS_LINE" ] && say "  状态行：$STATUS_LINE"
ARM_STATE=""
case "$STATUS_LINE" in
  *已解锁*) ARM_STATE="已解锁" ;;
  *已上锁*) ARM_STATE="已上锁" ;;
esac
case "$ARM_STATE" in
  已上锁) say "  ✓ PX4 已上锁" ;;
  已解锁) die "PX4 显示【已解锁】—— 手持俯仰前必须先上锁/断电，不能带桨上电测试" ;;
  *)      warn "未能从 /rosout 确认上锁状态 —— 请自己确认飞机未解锁" ;;
esac
VISION_MODE=$(printf '%s\n' "$STATUS_LINE" | grep -o 'mode=[0-9]' | head -1 || true)
say "  视觉上行：${VISION_MODE:-未知}（mode=3 才会给 PX4 发 331）"
SEND_HZ=$(printf '%s\n' "$STATUS_LINE" | grep -o '发=[0-9]*([0-9.]*Hz' | sed 's/.*(//; s/Hz$//' | head -1)
if [ -n "$SEND_HZ" ]; then
  say "  上行发送速率：${SEND_HZ} Hz"
  if ! hz_ge "$SEND_HZ" 1; then
    warn "vision_uplink 发送速率 ${SEND_HZ} Hz 接近 0 —— PX4 收不到视觉，先查 odom 延迟和位置闸门"
  fi
fi
case "$STATUS_LINE" in
  *已停发*) warn "状态行带【已停发】—— PX4 此刻收不到视觉" ;;
esac

if [ -f "$PARAM_CSV" ]; then
  say "  飞控参数（$PARAM_CSV）："
  grep -E 'EKF2_(HGT_REF|EV_CTRL|EV_DELAY|EVP_NOISE|EVV_NOISE|EVA_NOISE)' "$PARAM_CSV" \
    | sed 's/^/    /' || true
  SDLOG=$(awk -F, '$1 == "SDLOG_MODE" {print $2}' "$PARAM_CSV" 2>/dev/null | head -1)
  if [ -n "${SDLOG:-}" ]; then
    say "    SDLOG_MODE=$SDLOG"
    case "$SDLOG" in
      3|4) say "    ✓ 飞控从启动即写日志（手持未解锁也有 ULog）" ;;
      0|1|2) warn "SDLOG_MODE=$SDLOG 在未解锁时不写 ULog —— 手持测试会没有日志！测前用 QGC 改成 3 或 4 并重启飞控" ;;
    esac
  else
    warn "px4_params.csv 里没有 SDLOG_MODE —— 测前用 QGC 确认 SDLOG_MODE=3/4 且 SD 卡在位"
  fi
fi

say
say "[4/6] 磁盘与输出目录"
mkdir -p "$BAG_ROOT" || die "无法创建 $BAG_ROOT"
if [ -z "$NAME_ARG" ]; then
  NAME_ARG="pitch_test_$(date +%Y%m%d_%H%M%S)"
fi
case "$NAME_ARG" in
  *[!A-Za-z0-9_.-]*) die "包名只能包含字母、数字、下划线、点、横线：$NAME_ARG" ;;
esac
OUTDIR="$BAG_ROOT/$NAME_ARG"
[ -e "$OUTDIR" ] && die "输出目录已存在：$OUTDIR —— 换一个 --name"
FREE_GB=$(df -Pk "$BAG_ROOT" 2>/dev/null | awk 'NR==2 {print int($4/1024/1024)}')
[ -n "$FREE_GB" ] || FREE_GB=0
say "  $BAG_ROOT 可用 ${FREE_GB} GB（要求 ≥ ${MIN_FREE_GB} GB）"
[ "$FREE_GB" -ge "$MIN_FREE_GB" ] || die "磁盘空间不足"
say "  本次输出：$OUTDIR"
if [ "$(stat -c %d /tmp 2>/dev/null)" = "$(stat -c %d "$BAG_ROOT" 2>/dev/null)" ]; then
  warn "/tmp 与 bag 在同一块盘（$(df -P "$BAG_ROOT" 2>/dev/null | awk 'NR==2 {print $1}')）：bag 回写会拖住 IMU CSV 的行缓冲写；建议 IMU_CSV=/dev/shm/imu_raw_recover.csv"
fi

say
say "[5/6] 动作脚本"
PHASE_S=$(( MOVE_S + HOLD_S ))
TOTAL_S=$(( STATIC_HEAD_S + CYCLES * 4 * PHASE_S + STATIC_TAIL_S ))
if [ "$WITH_IMAGES" -eq 1 ]; then
  EST_GB=$(( TOTAL_S * 20 / 1024 + 1 ))
else
  EST_GB=1
fi
say "  总时长 ${TOTAL_S}s，预计 bag 约 ${EST_GB} GB（双目 30fps 原图约 20MB/s）"
cat <<EOF
  0~${STATIC_HEAD_S}s        静止（手离开）
  重复 ${CYCLES} 次，每次 $((4 * PHASE_S))s：
    快抬头 20~30° → 保持 ${HOLD_S}s → 快回正 → 保持 ${HOLD_S}s
    → 快低头 20~30° → 保持 ${HOLD_S}s → 快回正 → 保持 ${HOLD_S}s
  最后静止 ${STATIC_TAIL_S}s
  要点：只绕横轴俯仰，不要平移/偏航；快动在提示后 1~2 秒内完成，然后完全静止。
EOF

say
say "[6/6] PX4 ULog 提醒"
cat <<'EOF'
  ULog 不走 ROS，本脚本无法代录：
   · 测前确认飞控 SD 日志已开启（QGC 参数 SDLOG_MODE / SDLOG_PROFILE）
   · 测后停止 VINS 栈，用 QGC → Analyze Tools → Log Download
     下载覆盖本时段的 .ulg（vehicle_visual_odometry / estimator_* 都在里面）
EOF

if [ "$CHECK_ONLY" -eq 1 ]; then
  echo
  echo "✅ 预检通过（--check，不录制）"
  echo "   正式运行将写入：$OUTDIR/"
  echo "   预计时长 ${TOTAL_S}s，大小约 ${EST_GB} GB"
  exit 0
fi

# ---------------- 录制 ----------------
mkdir -p "$OUTDIR/meta" || die "无法创建 $OUTDIR/meta"
STAGE_LOG="$OUTDIR/meta/stages.csv"
printf 'label,wall_epoch,wall_iso,note\n' > "$STAGE_LOG"

# 话题清单：核心 + 诊断 + PX4 镜像；不存在的话题跳过
TOPICS=(
  /imu
  /vins_estimator/odometry
  /vins_estimator/imu_propagate
  /vins_estimator/point_cloud
  /vins_estimator/margin_cloud
  /vins_estimator/camera_pose
  /vins_estimator/extrinsic
  /vins_estimator/keyframe_pose
  /camera/infra1/camera_info
  /camera/infra2/camera_info
  /camera/infra1/metadata
  /camera/infra2/metadata
  /camera/stereo_module/parameter_updates
  /diagnostics
  /diag/att_delta
  /diag/vio_rpy
  /diag/grav_rp
  /diag/pos_norm
  /px4/local_position_ned
  /px4/estimator_status
  /px4/statustext
  /rosout
)
if [ "$WITH_IMAGES" -eq 1 ]; then
  TOPICS+=(/camera/infra1/image_rect_raw /camera/infra2/image_rect_raw)
fi

AVAILABLE=$(rostopic list 2>/dev/null || true)
REC_TOPICS=()
for t in "${TOPICS[@]}"; do
  if printf '%s\n' "$AVAILABLE" | grep -qxF "$t"; then
    REC_TOPICS+=("$t")
  else
    warn "话题不存在，跳过：$t"
  fi
done
[ "${#REC_TOPICS[@]}" -gt 0 ] || die "没有可录的话题"

say
if [ -t 0 ]; then
  echo "按 Enter 开始录制（Ctrl-C 取消）..."
  read -r _
else
  echo "非交互终端：5 秒后自动开始..."
  sleep 5
fi
for i in 5 4 3 2 1; do
  printf '\r  %d ...' "$i"
  printf '\a'
  sleep 1
done
printf '\r          \r'

# 记录日志切片起点
VINS_OFF=$(stat_size "$VINS_LOG")
CAM_OFF=$(stat_size "$CAM_LOG")
DIAG_OFF=$(stat_size "$DIAG_LOG")
IMU_OFF=$(stat_size "$IMU_LOG")
IMU_SUP_OFF=$(stat_size "$IMU_SUPER")
IMUCSV_OFF=$(stat_size "$IMU_CSV")
VISION_OFF=$(stat_size "$VISION_CSV")
PX4TEL_OFF=$(stat_size "$PX4TELEM_CSV")

# 录包进程放进独立 session：终端 Ctrl-C 不会直接打到 rosbag，
# 由本脚本显式发一次 SIGINT，避免连按 Ctrl-C 把 bag 写坏。
REC_LOG="$OUTDIR/meta/record.log"
MAXDUR=$(( TOTAL_S + 180 ))
pushd "$OUTDIR" >/dev/null || die "无法进入 $OUTDIR"
setsid nohup timeout -s INT "$MAXDUR" rosbag record -O "$NAME_ARG" \
  -b 128 --min-space=2G "${REC_TOPICS[@]}" > "$REC_LOG" 2>&1 &
REC_PID=$!
popd >/dev/null || true

FINALIZED=0
stop_recorder() {
  [ -n "${REC_PID:-}" ] || return 0
  if kill -0 "$REC_PID" 2>/dev/null; then
    echo
    echo "[record] 停止 rosbag（SIGINT，等它写完索引）..."
    kill -INT "$REC_PID" 2>/dev/null || true
    local i
    for i in $(seq 1 60); do
      kill -0 "$REC_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$REC_PID" 2>/dev/null; then
      warn "rosbag 30s 内未退出，继续等 10s（不要 kill -9）"
      sleep 10
    fi
  fi
  local i
  for i in $(seq 1 40); do
    [ -e "$OUTDIR/$NAME_ARG.bag.active" ] || break
    sleep 0.5
  done
  REC_PID=""
}

slice_log() {
  local src="$1" off="$2" dst="$3"
  [ -f "$src" ] || return 0
  local now
  now=$(stat -c %s "$src" 2>/dev/null || echo 0)
  if [ "$now" -ge "$off" ]; then
    tail -c +$(( off + 1 )) "$src" > "$dst" 2>/dev/null || true
  else
    cp "$src" "$dst" 2>/dev/null || true
    echo "ROTATED: $src (now=$now < offset=$off)" >> "$dst"
  fi
}

# 大 CSV 只取测试窗口：保留表头，丢掉切片起点那半行（最多丢一个样本）
slice_csv() {
  local src="$1" off="$2" dst="$3"
  [ -f "$src" ] || return 0
  head -1 "$src" > "$dst" 2>/dev/null || true
  local now
  now=$(stat -c %s "$src" 2>/dev/null || echo 0)
  if [ "$now" -gt "$off" ]; then
    tail -c +$(( off + 1 )) "$src" 2>/dev/null | tail -n +2 >> "$dst" 2>/dev/null || true
  fi
}

capture_meta() {
  local M="$OUTDIR/meta"
  mkdir -p "$M"

  slice_log "$VINS_LOG" "$VINS_OFF" "$M/vins_during.log"
  slice_log "$CAM_LOG"  "$CAM_OFF"  "$M/camera_during.log"
  slice_log "$DIAG_LOG" "$DIAG_OFF" "$M/diag_during.log"
  slice_log "$IMU_LOG"  "$IMU_OFF"  "$M/imu_bridge_during.log"
  slice_log "$IMU_SUPER" "$IMU_SUP_OFF" "$M/imu_super_during.log"
  slice_csv "$IMU_CSV"     "$IMUCSV_OFF" "$M/imu_raw_during.csv"
  slice_csv "$VISION_CSV"  "$VISION_OFF" "$M/vision_uplink_during.csv"
  slice_csv "$PX4TELEM_CSV" "$PX4TEL_OFF" "$M/px4_telemetry_during.csv"
  [ -f "$VINS_LOG" ] && head -n 120 "$VINS_LOG" > "$M/vins_head.log" 2>/dev/null || true

  for f in "$CAM_LOG" "$DIAG_LOG" "$PARAM_CSV" \
           "$LOG_DIR/pos_watch.csv" /root/calib_data/vins_output/vio.csv; do
    [ -f "$f" ] && cp -a "$f" "$M/" 2>/dev/null || true
  done

  [ -n "$VINS_CFG" ] && [ -f "$VINS_CFG" ] && cp -a "$VINS_CFG" "$M/" 2>/dev/null || true
  [ -d "$LIVE/config_x10" ] && cp -a "$LIVE/config_x10" "$M/" 2>/dev/null || true
  for f in px4_imu_node.py imu_clock_mapper.py vision_uplink.py vins_diag_relay.py \
           start_vins_pi.sh wait_fc_then_start_imu.sh record_pitch_test.sh; do
    [ -f "$LIVE/$f" ] && cp -a "$LIVE/$f" "$M/" 2>/dev/null || true
  done
  ( cd "$LIVE" && md5sum px4_imu_node.py imu_clock_mapper.py vision_uplink.py \
      vins_diag_relay.py start_vins_pi.sh config_x10/*.yaml 2>/dev/null ) \
    > "$M/md5sums.txt" 2>/dev/null || true

  rosparam dump "$M/rosparam.yaml" 2>/dev/null || true
  rosparam get /use_sim_time > "$M/use_sim_time.txt" 2>/dev/null || true
  rostopic list > "$M/rostopic_list.txt" 2>/dev/null || true
  rosnode list > "$M/rosnode_list.txt" 2>/dev/null || true
  {
    for t in /imu /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw \
             /vins_estimator/odometry /vins_estimator/point_cloud /px4/local_position_ned; do
      echo "=== $t"
      timeout 3 rostopic info "$t" 2>&1
      echo
    done
  } > "$M/topics_info.txt" 2>/dev/null || true

  # 这四个话题里 extrinsic 可能只在启动时发过；必须带 timeout，否则会永远等下去
  timeout 5 rostopic echo -n 1 /px4/estimator_status > "$M/px4_estimator_status_end.txt" 2>/dev/null || true
  timeout 5 rostopic echo -n 1 /px4/local_position_ned > "$M/px4_local_position_ned_end.txt" 2>/dev/null || true
  timeout 5 rostopic echo -n 1 /vins_estimator/odometry > "$M/vins_odometry_end.txt" 2>/dev/null || true
  timeout 5 rostopic echo -n 1 /vins_estimator/extrinsic > "$M/vins_extrinsic_end.txt" 2>/dev/null || true
  {
    echo "--- odometry ---"
    timeout 3 rostopic delay /vins_estimator/odometry 2>/dev/null
    echo "--- point_cloud ---"
    timeout 3 rostopic delay /vins_estimator/point_cloud 2>/dev/null
  } > "$M/vins_delay_end.txt" 2>/dev/null || true

  {
    echo "hostname: $(hostname)"
    echo "date: $(date -Is)"
    echo "uptime: $(cat /proc/uptime 2>/dev/null)"
    echo "temp_mC: $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)"
    echo "load: $(cat /proc/loadavg 2>/dev/null)"
    free -h 2>/dev/null
    echo "--- ps ---"
    ps -eo pid,etime,rss,args | grep -E '[v]ins_node|[p]x4_imu_node|[r]ealsense|[r]osbag record'
  } > "$M/system_state_end.txt" 2>/dev/null || true

  cp -a "$0" "$M/record_pitch_test.sh" 2>/dev/null || true
}

print_summary() {
  local bag="$OUTDIR/$NAME_ARG.bag"
  echo
  echo "================ 录制结束 ================"
  echo " 目录：$OUTDIR"
  echo " 实时性：odom |延迟| ${ODOM_LAG:-?}s，point_cloud |延迟| ${PC_LAG:-?}s（>0.5s 会被 uplink 停发）"
  if [ -f "$bag" ]; then
    echo " 包文件：$bag"
    echo " 大小：$(du -h "$bag" 2>/dev/null | awk '{print $1}')"
    if [ -f "$OUTDIR/meta/bag_info.txt" ]; then
      grep -E 'duration:|messages:|compression:' "$OUTDIR/meta/bag_info.txt" | head -3 | sed 's/^/   /'
      echo " 核心话题计数："
      grep -E '^[[:space:]]+/(imu|camera/infra1/image_rect_raw|camera/infra2/image_rect_raw|vins_estimator/odometry|vins_estimator/point_cloud)' \
        "$OUTDIR/meta/bag_info.txt" | sed 's/^/   /' || true
    fi
  else
    warn "没找到 $bag —— 录制可能失败，看 $OUTDIR/meta/record.log"
  fi
  echo
  echo " IMU 时钟映射（最后几条；scale/offset 与“时钟跳变”都是节点启动以来的累计值）："
  grep -a 'scale' "$OUTDIR/meta/imu_bridge_during.log" 2>/dev/null | tail -3 | sed 's/^/   /' \
    || echo "   （没有抓到，检查 $IMU_LOG）"
  echo
  echo " VINS 事件计数（仅本次测试窗口；初始化发生在启动时，看 vins_head.log）："
  for pat in 'Initialization finish' 'init begins' 'not enough' 'reset'; do
    local n
    n=$(grep -a -c "$pat" "$OUTDIR/meta/vins_during.log" 2>/dev/null || true)
    printf '   %-24s %s\n' "$pat" "${n:-0}"
  done
  local njump
  njump=$(grep -a -c '时钟基准跳变' "$OUTDIR/meta/imu_bridge_during.log" 2>/dev/null || true)
  echo " 本次窗口内 IMU 时钟基准跳变：${njump:-0} 次"
  if [ "${njump:-0}" -gt 0 ]; then
    warn "本次录制发生了 IMU 时钟基准跳变：stamp 可能已断，先别用这包做结论（看 meta/imu_raw_during.csv 的 stamp 列）"
  fi
  echo
  echo " 香橙派主机路径：/home/orangepi/vins_output/pitch_bags/$NAME_ARG"
  echo " 取回笔记本："
  echo "   scp -r orangepi@192.168.1.215:/home/orangepi/vins_output/pitch_bags/$NAME_ARG ~/drone/z_collect/pitch_bags/"
  echo " PX4 ULog 记得另下（QGC → Analyze Tools → Log Download）"
  echo "=========================================="
}

finalize() {
  [ "$FINALIZED" -eq 1 ] && return 0
  FINALIZED=1
  [ -n "${OUTDIR:-}" ] || return 0

  local i
  for i in $(seq 1 60); do
    [ -e "$OUTDIR/$NAME_ARG.bag.active" ] || break
    sleep 0.5
  done
  mark "record_end" "rosbag stopped"

  {
    echo "name=$NAME_ARG"
    echo "outdir=$OUTDIR"
    echo "start_wall=$(date -Is)"
    echo "total_planned_s=$TOTAL_S"
    echo "cycles=$CYCLES move_s=$MOVE_S hold_s=$HOLD_S"
    echo "with_images=$WITH_IMAGES"
    echo "vins_cfg=${VINS_CFG:-unknown}"
    echo "imu_hz=${IMU_HZ:-unknown} img1_hz=${IMG1_HZ:-unknown} img2_hz=${IMG2_HZ:-unknown} odom_hz=${ODOM_HZ:-unknown}"
    echo "odom_lag_s=${ODOM_LAG:-unknown} cloud_lag_s=${PC_LAG:-unknown}"
    echo "arm_state=${ARM_STATE:-unknown} vision_mode=${VISION_MODE:-unknown}"
  } > "$OUTDIR/meta/run_info.txt" 2>/dev/null || true

  local bag="$OUTDIR/$NAME_ARG.bag"
  if [ -f "$bag" ]; then
    rosbag info "$bag" > "$OUTDIR/meta/bag_info.txt" 2>&1 || true
    echo "$NAME_ARG" > "$BAG_ROOT/LATEST" 2>/dev/null || true
  else
    echo "missing bag: $bag" > "$OUTDIR/meta/bag_info.txt"
  fi

  capture_meta
  # 容器里是 root 建的；映射到主机后交给 orangepi（uid 1000 / gid 1001）管理
  chown -R 1000:1001 "$OUTDIR" 2>/dev/null || true
  chown 1000:1001 "$BAG_ROOT" 2>/dev/null || true
  chown 1000:1001 "$BAG_ROOT/LATEST" 2>/dev/null || true
  print_summary
}

on_exit() {
  local rc=$?
  trap - EXIT INT TERM HUP
  stop_recorder
  finalize
  exit "$rc"
}
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
trap 'on_exit' EXIT

# 确认录制进程活着
sleep 3
if ! kill -0 "$REC_PID" 2>/dev/null; then
  echo "[错误] rosbag record 启动失败，日志："
  cat "$REC_LOG" 2>/dev/null || true
  exit 1
fi
if [ ! -e "$OUTDIR/$NAME_ARG.bag.active" ] && [ ! -e "$OUTDIR/$NAME_ARG.bag" ]; then
  warn "还没看到 .bag.active，继续观察（日志：$REC_LOG）"
fi

mark "record_start" "rosbag started"
echo
echo "● 录制已启动：$OUTDIR/$NAME_ARG.bag"
echo "  下面跟着屏幕提示做动作。每次提示前会响铃（终端可能不发声）。"
echo

# ---------------- 动作脚本 ----------------
run_stage() {
  local label="$1" secs="$2" note="$3"
  mark "$label" "$note"
  printf '\a'
  echo
  echo "▶▶ $note"
  local start_ns end_ns now_ns left_ms i=0
  start_ns=$(date +%s%N)
  end_ns=$(( start_ns + secs * 1000000000 ))
  while :; do
    now_ns=$(date +%s%N)
    [ "$now_ns" -ge "$end_ns" ] && break
    left_ms=$(( (end_ns - now_ns) / 1000000 ))
    if [ -t 1 ]; then
      printf '\r    [%s] 剩余 %d.%01ds …   ' "$label" $(( left_ms / 1000 )) $(( (left_ms % 1000) / 100 ))
    else
      if [ $(( i % 50 )) -eq 0 ]; then
        printf '    [%s] 剩余 %d.%01ds\n' "$label" $(( left_ms / 1000 )) $(( (left_ms % 1000) / 100 ))
      fi
    fi
    sleep 0.1
    i=$(( i + 1 ))
  done
  [ -t 1 ] && printf '\r%*s\r' 70 ''
  echo "    ✓ ${label} 结束（$(date +%H:%M:%S)）"
}

run_stage "static_head" "$STATIC_HEAD_S" "静止 ${STATIC_HEAD_S} 秒：手离开，别碰桌子/线缆"
for i in $(seq 1 "$CYCLES"); do
  run_stage "c${i}_up_move"   "$MOVE_S" "第 ${i}/${CYCLES} 次 · 快速抬头 20~30°（相机光轴向上）"
  run_stage "c${i}_up_hold"   "$HOLD_S" "保持抬头，别动"
  run_stage "c${i}_level1"    "$MOVE_S" "快速回正到水平"
  run_stage "c${i}_level1_hold" "$HOLD_S" "保持水平，别动"
  run_stage "c${i}_down_move" "$MOVE_S" "快速低头 20~30°（相机光轴向下）"
  run_stage "c${i}_down_hold" "$HOLD_S" "保持低头，别动"
  run_stage "c${i}_level2"    "$MOVE_S" "快速回正到水平"
  run_stage "c${i}_level2_hold" "$HOLD_S" "保持水平，别动"
done
run_stage "static_tail" "$STATIC_TAIL_S" "静止 ${STATIC_TAIL_S} 秒：手离开"
mark "protocol_end" "动作结束"

echo
echo "动作结束，停止录制并写索引..."
exit 0
