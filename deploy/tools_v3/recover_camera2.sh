#!/bin/bash
# 相机 wedge（EBUSY 刷屏）恢复流程 —— 2026-09-28 第二次，上次这套路走通过。
# 原理：进程退出会释放 libusb 对接口的 claim，然后 initial_reset 重拉就能拿回相机。
# 全程 SIGINT，绝不 kill -9（硬杀会把相机彻底搞卡，只能靠物理拔插）。
set -u
TOOLS=/root/xu_drone/live
LOG=/tmp/vins_pi

echo "########## A. 取证（动手之前先看清）##########"

echo "=== A1. 供电来源（笔记本上电池供电曾让掉线率涨到 35 倍）==="
found=0
for p in /sys/class/power_supply/*; do
  [ -d "$p" ] || continue
  found=1
  echo "  $(basename $p): type=$(cat $p/type 2>/dev/null) status=$(cat $p/status 2>/dev/null) capacity=$(cat $p/capacity 2>/dev/null)"
done
[ "$found" = 0 ] && echo "  （没有 power_supply 设备 ⇒ 无电池，是直流供电）"

echo
echo "=== A2. 相机 wedge 时间线（camera.log 里的关键指纹）==="
grep -nE "Asic Temperature|No such device|watchdog|reset_endpoint|MIPI|RealSense Node Is Up|SELECTED BASE" "$LOG/camera.log" | head -14
echo "  EBUSY 条数: $(grep -c 'Device or resource busy' "$LOG/camera.log" 2>/dev/null)"
echo "  日志首行时间: $(head -3 "$LOG/camera.log" | grep -oE '^ *[0-9]+/[0-9]+ [0-9:,]+' | head -1)"
echo "  日志末行时间: $(tail -3 "$LOG/camera.log" | grep -oE '^ *[0-9]+/[0-9]+ [0-9:,]+' | tail -1)"

echo
echo "=== A3. 相机真的还在 USB 总线上吗（上次靠 lsusb，容器里没这命令，白查）==="
hit=0
for d in /sys/bus/usb/devices/*/; do
  v=$(cat "$d/idVendor" 2>/dev/null)
  if [ "$v" = "8086" ]; then
    hit=1
    echo "  ★ ${d}  idProduct=$(cat $d/idProduct 2>/dev/null)  speed=$(cat $d/speed 2>/dev/null)  busnum=$(cat $d/busnum 2>/dev/null) devnum=$(cat $d/devnum 2>/dev/null)"
  fi
done
[ "$hit" = 0 ] && echo "  ★★ 总线上找不到 8086: 设备 —— 软件救不回，必须物理拔插"
echo "  端口 2-1 目录: $([ -d /sys/bus/usb/devices/2-1 ] && echo 在 || echo 没了)"

echo
echo "########## B. 恢复 ##########"
echo "=== B1. 优雅停整栈（SIGINT）==="
bash "$TOOLS/stop_vins_pi.sh" 2>&1 | sed 's/^/  /'
sleep 12

echo "=== B2. 复查残留（相机节点退出要十几秒）==="
pgrep -af "roscore|[r]ealsense|[n]odelet|[v]ins_node|[f]oxglove" | sed 's/^/  /' || echo "  干净"

echo "=== B3. 重拉相机（initial_reset:=true）==="
# ★ v2 修复：ROS 的 profile.d/10.roslaunch.sh 第 3 行是 `if [ ! "$ROS_MASTER_URI" ]`（裸引用），
#   与本脚本第 5 行的 set -u 冲突 → 未定义变量会让脚本当场退出（2026-10-03 实测踩到，
#   后果是栈已停、相机没重拉）。下面 set +u 只关掉这一段的严格模式，取证段仍保持 set -u。
set +u
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
[ -f /tmp/vins_pi/camera.log ] && mv /tmp/vins_pi/camera.log /tmp/vins_pi/camera.log.wedge2
roslaunch realsense2_camera rs_camera.launch \
  enable_infra1:=true enable_infra2:=true enable_color:=false enable_depth:=false \
  infra_width:=640 infra_height:=480 infra_fps:=30 initial_reset:=true \
  > "$LOG/camera.log" 2>&1 &
for _ in $(seq 1 60); do
  grep -q "RealSense Node Is Up" "$LOG/camera.log" 2>/dev/null && break
  sleep 1
done
if grep -q "RealSense Node Is Up" "$LOG/camera.log" 2>/dev/null; then
  echo "  ✓ 相机起来了"
else
  echo "  ✗ 没起来，日志尾部："; tail -8 "$LOG/camera.log" | sed 's/^/    /'
fi

echo "=== B4. 发射器守卫（必须在相机之后立刻确认它还活着）==="
pgrep -af "[e]mitter_guard" | sed 's/^/  /' || echo "  ★ 守卫不在，得手动拉起"

echo "=== B5. 等 14 秒让自动曝光收敛，再量角点数 ==="
sleep 14
timeout 40 python3 "$TOOLS/feat_watch.py" 8 2>&1 | sed 's/^/  /'
