#!/bin/bash
# 只拉【发射器守卫 + 相机】，不碰 vins_node —— 用户要一边调视角一边看角点数。
#
# ★ 本脚本刻意【不写 set -u】：
#   ROS 的 /opt/ros/noetic/etc/catkin/profile.d/10.roslaunch.sh 第 3 行会引用
#   ROS_MASTER_URI，该变量未定义。set -u 会让 source 直接失败（实测 exit 1），
#   而且报错信息看起来像"ROS 装坏了"，其实是自己的脚本把 shell 调严了。
TOOLS=/root/xu_drone/live
LOG=/tmp/vins_pi

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

echo "=== 1. roscore ==="
if pgrep -x roscore >/dev/null; then echo "  ✓ 已在跑"; else roscore > "$LOG/roscore.log" 2>&1 & sleep 4; echo "  已拉起"; fi

echo "=== 2. 发射器守卫（必须先起：相机启动瞬间 librealsense 会把 emitter_enabled 设回 1）==="
if pgrep -f "[e]mitter_guard" >/dev/null; then
  echo "  ✓ 已在跑"
else
  python3 -u "$TOOLS/emitter_guard.py" > "$LOG/emitter_guard.log" 2>&1 &
  echo "  已拉起"
fi

echo "=== 3. 相机（initial_reset:=true，设备已在总线上，软件路径可救）==="
[ -f "$LOG/camera.log" ] && mv "$LOG/camera.log" "$LOG/camera.log.wedge2"
roslaunch realsense2_camera rs_camera.launch \
  enable_infra1:=true enable_infra2:=true enable_color:=false enable_depth:=false \
  infra_width:=640 infra_height:=480 infra_fps:=30 initial_reset:=true \
  > "$LOG/camera.log" 2>&1 &
for _ in $(seq 1 90); do
  grep -q "RealSense Node Is Up" "$LOG/camera.log" 2>/dev/null && break
  sleep 1
done
if grep -q "RealSense Node Is Up" "$LOG/camera.log" 2>/dev/null; then
  echo "  ✓ 相机起来了"
else
  echo "  ✗ 没起来，日志尾部："; tail -8 "$LOG/camera.log" | sed 's/^/    /'; exit 1
fi

echo "=== 4. 等 14 秒让自动曝光收敛（实测 AE 要 ~10 秒，不等会量到过渡态）==="
sleep 14

echo "=== 5. 发射器最终确认 ==="
timeout 20 rostopic echo -n 1 /camera/infra1/emitter_enabled 2>/dev/null | head -2 | sed 's/^/  /'
grep -E "★|心跳" "$LOG/emitter_guard.log" 2>/dev/null | tail -3 | sed 's/^/  /'

echo "=== 6. 当前角点数（调视角时另开 feat_live.sh 看实时）==="
timeout 30 python3 "$TOOLS/feat_watch.py" 6 2>&1 | tail -6 | sed 's/^/  /'
