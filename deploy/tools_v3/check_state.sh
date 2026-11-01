#!/bin/bash
# 一轮跑起来之后的体检：每一条都必须贴住被测量的信号本身。
# ★ 教训来源：裸 `docker exec` 不带 ROS 环境，rostopic/grab_frame 全都 ModuleNotFound，
#   而 grep 过滤后【什么都不打印】看起来像"话题没数据"。所以本脚本自己 source。
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
LOG=/tmp/vins_pi/vins.log

echo "=== 1. 话题频率（每条 8 秒窗口）==="
for t in /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /imu /vins_estimator/odometry; do
  r=$(timeout 10 rostopic hz "$t" 2>&1 | grep -m1 "average rate")
  printf "  %-40s %s\n" "$t" "${r:-★ 没数据}"
done

echo
echo "=== 2. 视觉到底出没出力：/diag/pos_norm 三帧 ==="
echo "    格式  x=|VIO|  y=|纯IMU积分|  z=|两者差|     —— x≈y 就是视觉归零"
timeout 10 python3 /root/xu_drone/live/pos_watch.py 6 2>&1 | tail -6

echo
echo "=== 3. numerical unstable 出在哪（源码行 + 时间分布）==="
echo -n "  总数: "; grep -c "numerical unstable" "$LOG"
echo -n "  首条: "; grep -m1 -o "\[INFO\] \[[0-9.]*\]" "$LOG" >/dev/null; grep -m1 "numerical unstable" "$LOG" | head -c 200
echo -n "  末条: "; grep "numerical unstable" "$LOG" | tail -1 | head -c 200
echo
echo "  按分钟分布（日志里的 ROS 时间戳折算）:"
grep "numerical unstable" "$LOG" | grep -o "\[[0-9]\{10\}\.[0-9]*\]" | tr -d '[]' \
  | awk '{t=int($1); c[t]++} END {for (k in c) printf "    %s  %d 条\n", strftime("%H:%M:%S", k, 1), c[k]}' | sort

echo
echo "=== 4. vins.log 是否还在长（最后修改距今）==="
echo "  $(( $(date +%s) - $(stat -c %Y "$LOG") )) 秒前"
echo -n "  当前日志行数: "; wc -l < "$LOG"

echo
echo "=== 5. 进程 + 发射器 ==="
pgrep -af "[v]ins_node" | head -2
pgrep -af "[e]mitter_guard" | head -1
