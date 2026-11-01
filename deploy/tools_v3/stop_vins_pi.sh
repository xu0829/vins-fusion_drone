#!/bin/bash
# 独立收尾：把 start_vins_pi.sh 拉起来的整栈关掉。
#
# 为什么需要它：start_vins_pi.sh 的 trap 只管 SIGTERM/SIGINT，
# 而 feat 闸门不过时是 `exit 1` 退出的 —— **trap 不触发，相机/roscore/守卫全都留着**。
# （留着是有意的，方便边调视角边看 feat_watch；但那就得有个东西能收尾，
#   而这时启动器已经退了，`pkill -f start_vins_pi` 什么也匹配不到。）
#
# 用法（容器内）：bash /root/xu_drone/live/stop_vins_pi.sh
# 全程 SIGINT/SIGTERM，**绝不用 kill -9** —— 硬杀会把相机搞卡，
# 只能靠 USB 授权位复位救（见记忆 ros1-stale-registration-trap / camera-wedge）。
set -u

echo "[stop] 优雅关闭中（全程 SIGINT，不用 kill -9）..."

for pat in vins_diag_relay vins_node px4_imu_node wait_fc_then_start foxglove_bridge emitter_guard; do
  pkill -INT -f "$pat" 2>/dev/null && echo "  → 已通知 $pat"
done
pkill -INT -f "roslaunch realsense2_camera" 2>/dev/null && echo "  → 已通知 相机"
sleep 10
pkill -INT -f roscore 2>/dev/null && echo "  → 已通知 roscore"
sleep 4

echo "[stop] 复查还剩什么："
left=$(pgrep -af "roscore|realsense|emitter_guard|vins_node|px4_imu|foxglove|vins_diag_relay" 2>/dev/null | grep -v "pgrep\|stop_vins_pi")
if [ -z "$left" ]; then
  echo "  ✓ 干净"
else
  echo "$left" | sed 's/^/    /'
  echo "  ⚠ 上面这些还在。**再等 10 秒**，别急着 kill -9 ——"
  echo "    相机节点退出有时要十几秒，硬杀会把它搞卡。"
fi
