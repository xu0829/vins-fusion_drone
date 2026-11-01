#!/usr/bin/env bash
set -Eeuo pipefail

# Record a stereo/IMU AprilGrid bag, then run Kalibr without changing VINS files.

# docker exec starts a non-login shell; load the ROS workspaces when present.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [[ -f /root/catkin_ws/devel/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /root/catkin_ws/devel/setup.bash
fi

usage() {
  cat <<'EOF'
Usage:
  record_and_kalibr_aprilgrid.sh record --out DIR [--seconds N]
  record_and_kalibr_aprilgrid.sh calibrate --bag BAG --target TARGET.yaml \
      --camchain CAMCHAIN.yaml --imu IMU.yaml
  record_and_kalibr_aprilgrid.sh all --out DIR --camchain CAMCHAIN.yaml --imu IMU.yaml
EOF
}

die() { echo "[ERROR] $*" >&2; exit 1; }
need_cmd() { command -v "$1" >/dev/null 2>&1 || die "缺少命令: $1"; }

IMU_TOPIC=${IMU_TOPIC:-/imu}
CAM0_TOPIC=${CAM0_TOPIC:-/camera/infra1/image_rect_raw}
CAM1_TOPIC=${CAM1_TOPIC:-/camera/infra2/image_rect_raw}
SECONDS_TO_RECORD=${SECONDS_TO_RECORD:-180}

check_ros() {
  need_cmd rostopic
  need_cmd rosbag
  rostopic list >/dev/null 2>&1 || die "ROS master 不可达"
  for topic in "$IMU_TOPIC" "$CAM0_TOPIC" "$CAM1_TOPIC"; do
    rostopic list | grep -Fxq "$topic" || die "缺少话题: $topic"
  done
}

write_target() {
  local target=$1
  if [[ -e "$target" ]]; then
    echo "[info] 使用已有 AprilGrid 配置: $target"
    return
  fi
  cat >"$target" <<'EOF'
target_type: 'aprilgrid'
tagCols: 5
tagRows: 5
tagSize: 0.03191
tagSpacing: 0.3
EOF
  echo "[info] 已创建 AprilGrid 配置: $target"
}

record_bag() {
  local out_dir=$1 duration=${2:-$SECONDS_TO_RECORD}
  mkdir -p "$out_dir"
  check_ros
  need_cmd timeout
  echo "[preflight] IMU 频率（约 10 秒）:"
  timeout 12s rostopic hz "$IMU_TOPIC" || true
  echo "[preflight] 图像频率（约 10 秒）:"
  timeout 12s rostopic hz "$CAM0_TOPIC" || true
  timeout 12s rostopic hz "$CAM1_TOPIC" || true
  echo "[record] 将录制 ${duration}s；请让 AprilGrid 覆盖画面并做缓慢、多方向运动。"
  rosbag record --duration="$duration" --lz4 -O "$out_dir/aprilgrid_imu.bag" \
    "$IMU_TOPIC" "$CAM0_TOPIC" "$CAM1_TOPIC"
}

calibrate() {
  local bag=$1 target=$2 camchain=$3 imu_yaml=$4
  [[ -f "$bag" ]] || die "bag 不存在: $bag"
  [[ -f "$target" ]] || die "AprilGrid 配置不存在: $target"
  [[ -f "$camchain" ]] || die "相机链文件不存在: $camchain"
  [[ -f "$imu_yaml" ]] || die "IMU YAML 不存在: $imu_yaml"
  need_cmd kalibr_calibrate_imu_camera
  mkdir -p "$(dirname "$bag")/kalibr_output"
  echo "[kalibr] 只读输入，不修改 VINS 配置。"
  kalibr_calibrate_imu_camera --target "$target" --cam "$camchain" \
    --imu "$imu_yaml" --bag "$bag" --dont-show-report
}

[[ $# -gt 0 ]] || { usage; exit 2; }
mode=$1; shift
out_dir=""; bag=""; target=""; camchain=""; imu_yaml=""; duration=$SECONDS_TO_RECORD
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) out_dir=${2:?missing value}; shift 2 ;;
    --seconds) duration=${2:?missing value}; shift 2 ;;
    --bag) bag=${2:?missing value}; shift 2 ;;
    --target) target=${2:?missing value}; shift 2 ;;
    --camchain) camchain=${2:?missing value}; shift 2 ;;
    --imu) imu_yaml=${2:?missing value}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数: $1" ;;
  esac
done

case "$mode" in
  record)
    [[ -n "$out_dir" ]] || die "record 需要 --out DIR"
    record_bag "$out_dir" "$duration"
    ;;
  calibrate)
    [[ -n "$bag" && -n "$camchain" && -n "$imu_yaml" ]] || die "calibrate 需要 --bag、--camchain、--imu"
    [[ -n "$target" ]] || target="$(dirname "$bag")/aprilgrid_5x5_30mm_screen.yaml"
    write_target "$target"
    calibrate "$bag" "$target" "$camchain" "$imu_yaml"
    ;;
  all)
    [[ -n "$out_dir" && -n "$camchain" && -n "$imu_yaml" ]] || die "all 需要 --out、--camchain、--imu"
    target="$out_dir/aprilgrid_5x5_30mm_screen.yaml"
    write_target "$target"
    record_bag "$out_dir" "$duration"
    calibrate "$out_dir/aprilgrid_imu.bag" "$target" "$camchain" "$imu_yaml"
    ;;
  *) usage; die "模式必须是 record、calibrate 或 all" ;;
esac
