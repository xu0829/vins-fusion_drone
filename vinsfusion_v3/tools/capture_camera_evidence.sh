#!/usr/bin/env bash
# 在容器内抓取 D430 自检证据：设备信息、话题频率、各一帧红外图。
# 前提：realsense2_camera 节点已在跑，且 roscore 可访问。
# 用法（容器内，已 source /opt/ros/noetic/setup.bash）：bash tools/capture_camera_evidence.sh [输出目录]
set -u

OUT="${1:-/root/vinsfusion/tools/evidence}"
mkdir -p "$OUT"
STAMP=$(date +%Y%m%d_%H%M%S)
TXT="$OUT/camera_selfcheck_${STAMP}.txt"

{
  echo "D430 相机自检证据 —— 生成于 $(date -Iseconds)"
  echo "容器: $(hostname)   内核: $(uname -r)"
  echo "ROS: $ROS_DISTRO"
  echo
  echo "===== 1. USB 层识别 ====="
  lsusb | grep -i -E "intel|realsense" || echo "(lsusb 未匹配到 Intel/RealSense)"
  echo
  echo "--- 设备节点 ---"
  ls -l /dev/video* 2>/dev/null
  echo
  echo "--- USB 速率（5000M = USB3.0）---"
  for d in /sys/bus/usb/devices/*/; do
    if [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ]; then
      echo "  $(basename $d): idProduct=$(cat $d/idProduct 2>/dev/null) speed=$(cat $d/speed 2>/dev/null)M product=$(cat $d/product 2>/dev/null)"
      echo "    当前速率: $(cat $d/current_speed 2>/dev/null)"
    fi
  done
  echo
  echo "===== 2. 话题清单 ====="
  rostopic list 2>/dev/null | grep -E "infra|imu" || echo "(未取到)"
  echo
  echo "===== 3. 频率实测（每个话题测 12 秒）====="
  for t in /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw; do
    echo "--- $t ---"
    timeout 12 rostopic hz "$t" 2>&1 | grep -vE "^$" | tail -5
    echo
  done
  echo "===== 4. 相机内参（硬件标定，用于与 Kalibr 结果对照）====="
  for t in /camera/infra1/camera_info /camera/infra2/camera_info; do
    echo "--- $t ---"
    timeout 8 rostopic echo -n1 "$t" 2>/dev/null | grep -A3 -E "^(width|height|distortion_model|K:|D:)" | head -20
    echo
  done
  echo "===== 5. IR 发射器状态 ====="
  timeout 8 rostopic echo -n1 /camera/stereo_module/parameter_updates 2>/dev/null | grep -B1 -A1 "emitter" | head -20 || echo "(未取到)"
} > "$TXT" 2>&1

echo "文本证据: $TXT"

# 抓帧：用 cv_bridge 把一帧写成 PNG
python3 - "$OUT" "$STAMP" <<'PY'
import sys, rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
out, stamp = sys.argv[1], sys.argv[2]
got = {}
def mk(topic, name):
    def cb(msg):
        try:
            img = cv_bridge.CvBridge().imgmsg_to_cv2(msg, 'mono8')
            path = f"{out}/{name}_{stamp}.png"
            cv2.imwrite(path, img)
            got[name] = (path, img.shape, int(img.mean()), int(img.min()), int(img.max()))
        except Exception as e:
            got[name] = ('ERROR: %s' % e,)
        if len(got) >= 2:
            rospy.signal_shutdown('done')
    rospy.Subscriber(topic, Image, cb, queue_size=1)
rospy.init_node('camera_evidence', anonymous=True, disable_signals=True)
mk('/camera/infra1/image_rect_raw', 'infra1')
mk('/camera/infra2/image_rect_raw', 'infra2')
try:
    rospy.spin()
except rospy.ROSInterruptException:
    pass
for k, v in sorted(got.items()):
    print(f"  {k}: {v}")
PY

echo
echo "输出目录内容："
ls -la "$OUT"
