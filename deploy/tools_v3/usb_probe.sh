#!/bin/bash
# 相机 USB 链路体检 —— **修好之后拿它验证**，别靠"能出图"猜。
#
# 为什么需要它：2026-09-28 实测发现相机从 USB3(5000M) 掉到了 USB2(480M)，
# 而"话题有数据""能出图"完全看不出来 —— 480M 也跑得动，只是余量只剩一点。
# 基线是 USB3 5000Mbps。低于这个数就是**降级运行**，测出来的东西不能和基线比。
#
# ★ 判据是 sysfs 里的 speed，不是 /dev/video*（librealsense 走 libusb，
#   会把 uvcvideo 摘掉，/dev/video* 消失是正常行为）。
set +u
echo "=== 相机 USB 链路 ==="
found=0
for d in /sys/bus/usb/devices/*/; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
  pid=$(cat "$d/idProduct" 2>/dev/null)
  [ "$pid" = "0ad4" ] || continue
  found=1
  spd=$(cat "$d/speed" 2>/dev/null)
  bus=$(cat "$d/busnum" 2>/dev/null); dev=$(cat "$d/devnum" 2>/dev/null)
  echo "  设备 $d"
  echo "    速率  $spd Mbps   （基线 5000 = USB3 SuperSpeed）"
  echo "    位置  bus $bus / dev $dev"
  echo "    驱动  $(basename $(readlink -f "$d/../.." 2>/dev/null))"
  if [ "$spd" = "5000" ]; then
    echo "    ✓ 在 USB3 上，与基线一致"
  elif [ "$spd" = "480" ]; then
    echo "    ✗★ 只跑在 USB2（480M）—— 降级运行，别在这上面做实验"
    echo "       物理排查顺序：换 USB3 口 → 换线 → 看供电"
  else
    echo "    ⚠ 速率 $spd 不是预期值（5000/480），先查清楚"
  fi
done
[ "$found" = "0" ] && echo "  ✗ 总线上找不到 8086:0ad4（相机不在？）"

echo
echo "=== 两条 USB3 口上有没有设备（空 = 相机没插在 USB3 口上）==="
for b in 1 2 7 8; do
  n=$(lsusb -s $b: 2>/dev/null | wc -l)
  printf "  bus %-2s : %s 个设备\n" "$b" "$n"
done

echo
echo "=== 相机话题频率（能出图 ≠ 链路健康，两个都要看）==="
source /opt/ros/noetic/setup.bash 2>/dev/null
source /root/catkin_ws/devel/setup.bash 2>/dev/null
for tp in /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw; do
  hz=$(timeout 8 rostopic hz -w 20 "$tp" 2>/dev/null | grep -m1 "average rate" || echo "（没测到）")
  printf "  %-34s %s\n" "$tp" "$hz"
done

echo
echo "=== 内核 USB 事件（相机那条线，看有没有再掉）==="
dmesg 2>/dev/null | grep -E "usb [0-9]-1" | grep -vE "Found UVC" | tail -8
