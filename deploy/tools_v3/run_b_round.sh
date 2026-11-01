#!/bin/bash
# B 轮：IMU 噪声参数 ×10（acc_n 0.1→1.0, gyr_n 0.01→0.1），其余全部与 A 轮相同。
#
# 两个设计要点：
#   1. 【相机和发射器守卫不动】。stop_vins_pi.sh 会把相机一起停掉，重拉要再来一次
#      initial_reset（多一次 USB 风险），而且曝光会重新收敛 —— 那就变成"换了相机会话"
#      而不是"只换了参数"。所以这里只 SIGINT vins_node 和 relay。
#   2. 【记录器必须在 vins_node 之前启动】。A 轮我只从第 2 分钟起才有数据，
#      分不清"初始化那一刻姿态就错了"和"零偏慢慢把它漂出去"。
set +u          # ★ 不要 set -u：ROS 的 profile.d/10.roslaunch.sh 引用未定义的 ROS_MASTER_URI
TOOLS=/root/xu_drone/live
LOG=/tmp/vins_pi
X10=$TOOLS/config_x10/d430_px4_live_refnoise_x10.yaml
BLOG=$LOG/launcher_B.log

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

echo "=== 0. x10 配置在位吗（md5 应为 2d3e96f1dc9f911c0af13c38b2ea61d8）==="
ls -l "$X10" 2>/dev/null && md5sum "$X10" || { echo "  ✗ 配置不在，中止"; exit 1; }

echo "=== 1. 只停 vins_node 和 relay（相机/守卫/roscore 全留着）==="
pkill -INT -f "[v]ins_node"      2>/dev/null && echo "  → 已通知 vins_node"  || echo "  vins_node 本来没跑"
pkill -INT -f "[v]ins_diag_relay" 2>/dev/null && echo "  → 已通知 relay"      || echo "  relay 本来没跑"
sleep 6

echo "=== 2. 先开记录器（★ 在 vins_node 之前）==="
rm -f "$LOG/pos_watch.csv"
nohup timeout 1200 python3 "$TOOLS/pos_watch.py" 1150 > "$LOG/pos_watch_B.log" 2>&1 &
PW=$!
sleep 6
if kill -0 "$PW" 2>/dev/null; then
  echo "  ✓ pos_watch 活着（pid $PW），正在等 /diag/* 出现"
else
  echo "  ✗ pos_watch 已退出，日志："; tail -6 "$LOG/pos_watch_B.log" | sed 's/^/    /'
fi

echo "=== 3. 起 B 轮（CONFIG = x10）==="
LAUNCH_EPOCH=$(date +%s)          # ★ 记下启动时刻，第 5 步用它判零偏是不是本轮的
nohup env CONFIG="$X10" bash "$TOOLS/start_vins_pi.sh" > "$BLOG" 2>&1 &
for _ in $(seq 1 100); do
  grep -qE "全栈已拉起|✗" "$BLOG" 2>/dev/null && break
  sleep 2
done
tail -24 "$BLOG" | sed 's/^/  /'

echo
echo "=== 4. ★ 验证真的加载的是 x10：不看自己的变量，看进程命令行 ==="
pgrep -af "[v]ins_node"

echo "=== 5. B 轮抽到的零偏（约 10 秒后）==="
# ★ 判据是【这行的 ROS 时间戳晚于本轮启动时刻】，不是"文件有没有变长"。
#   踩过两个坑：
#     (a) 最早直接 grep，读到的是【上一轮】留下的那行，把 A 轮的零偏报成 B 轮的；
#     (b) 改成比 mtime/行数，结果 wait 循环一看行已在就立刻 break，
#         前后两次采样相隔 0 秒 ⇒ mtime 和行数必然相等 ⇒【每轮都误报"是上一轮的"】。
#   ROS 时间戳就是香橙派自己的 UTC epoch，和 date +%s 同一把尺子，可以直接比。
BIAS_LINE=""
for _ in $(seq 1 40); do
  BIAS_LINE=$(grep -m1 "gyroscope bias initial calibration" "$LOG/vins.log" 2>/dev/null)
  [ -n "$BIAS_LINE" ] && break
  sleep 1
done
BIAS_TS=$(printf '%s' "$BIAS_LINE" | sed -n 's/.*\[\([0-9]\{9,\}\)\..*/\1/p')
if [ -n "$BIAS_LINE" ] && [ -n "$BIAS_TS" ] && [ "$BIAS_TS" -ge "$LAUNCH_EPOCH" ]; then
  echo "  ✓ 时间戳 $BIAS_TS ≥ 本轮启动 $LAUNCH_EPOCH —— 确认是【本轮】抽的"
else
  echo "  ✗★ 时间戳「${BIAS_TS:-取不到}」早于本轮启动 $LAUNCH_EPOCH"
  echo "     ⇒ 下面这行是【上一轮】的，不是本轮的，别拿它下结论"
fi
printf '%s\n' "$BIAS_LINE" | sed 's/.*calibration *//; s/\x1b\[[0-9;]*m//g' | sed 's/^/  /'
echo "  对照 A 轮: -0.000339504  0.000610363  -0.00143139   (合成倾斜 2.41°/分)"
echo "  对照真值 : +0.000035     +0.000017     -0.000068"
echo -n "  numerical unstable: "; grep -c "numerical unstable" "$LOG/vins.log"
