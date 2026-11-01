#!/bin/bash
# 实时角点数监视器 —— 调飞机视角时盯着它。
# ★ 不要写 set -u（同 relaunch_camera.sh，ROS 的 profile.d 会引用未定义变量）
#
# 判据（VINS 的 max_cnt=150 / min_dist=30，与 feat_watch 同参数）：
#   infra1 ≥ 85   → 好，可以起 VINS
#   infra1  80~85 → 刚够，记进结论
#   infra1 <  80  → 视觉饿死，别起
# 两边都要看：infra2 单独高没用，VINS 要两个相机同时有货。
set +u
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

echo "  时间      infra1 角点/亮度      infra2 角点/亮度     判据"
while true; do
  out=$(timeout 20 python3 /root/xu_drone/live/feat_watch.py 3 2>/dev/null | grep "infra1:" | tail -1)
  if [ -z "$out" ]; then
    echo "  ---- 没读到图像（相机可能没起来）----"
  else
    n1=$(echo "$out" | grep -oE 'infra1: *[0-9-]+' | grep -oE '[0-9-]+$')
    if   [ "${n1:--1}" -ge 85 ] 2>/dev/null; then v="✓ 好，可以起 VINS"
    elif [ "${n1:--1}" -ge 80 ] 2>/dev/null; then v="~ 刚够"
    else v="✗ 饿死，别起"; fi
    echo "  $out   $v"
  fi
done
