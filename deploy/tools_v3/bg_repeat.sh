#!/bin/bash
# 反复重启 vins_node，收集它每次初始化解出来的陀螺零偏 —— 判定"是噪声抽样还是稳定错值"。
#
# 为什么这个实验一锤定音：
#   solveGyroscopeBias 的解（源码 initial_aligment.cpp:20-37）是
#       delta_bg = A.ldlt().solve(b),  b = Σ tmp_Aᵀ · 2·(delta_q⁻¹ · q_ij).vec()
#   其中 tmp_A = ∂ΔR/∂bg ≈ -dt·I，所以 delta_bg ≈ -(视觉旋转残差)/dt 的加权平均。
#   ★ 这是个【微分器】：视觉旋转的小误差被除以 dt（0.1 秒）放大 10 倍。
#   而飞机静止 ⇒ 视差≈0 ⇒ 本质矩阵是从噪声里解出来的 ⇒ q_ij 是垃圾旋转。
#
#   于是两种可能，读数会直接分开：
#     · 若零偏是噪声抽样  → 每次重启解出来的值【每次都不一样】，散布和陀螺噪声同量级
#     · 若有什么在系统性拉它 → 每次都很接近同一个值
#
# 对照真值（gyro_bias.py 30 秒 / 4437 样本，2026-09-28 13:37）：
#     x=+0.000035  y=+0.000017  z=-0.000068  rad/s
#
# 用法（容器内）：bash bg_repeat.sh [次数，默认 5]
set -u

CONFIG=${CONFIG:-/root/catkin_ws/src/VINS-Fusion/config/d430_px4/d430_px4_live_refnoise.yaml}
LOG=/tmp/vins_pi
N=${1:-5}

echo "=================================================================="
echo "  重启 vins_node $N 次，只取初始化那一行（约 1 秒出结果），然后 SIGINT"
echo "  实测真值 x=+0.000035  y=+0.000017  z=-0.000068   rad/s"
echo "=================================================================="

for i in $(seq 1 "$N"); do
  pkill -INT -f "[v]ins_node" 2>/dev/null
  sleep 3
  f="$LOG/bg_probe_$i.log"
  rosrun vins vins_node "$CONFIG" > "$f" 2>&1 &
  for _ in $(seq 1 40); do
    grep -q "gyroscope bias initial calibration" "$f" 2>/dev/null && break
    sleep 0.5
  done
  line=$(grep -m1 "gyroscope bias initial calibration" "$f" 2>/dev/null \
         | sed 's/.*calibration *//; s/\x1b\[[0-9;]*m//g')
  # 顺带量一下初始化窗口有多长（init begins → Initialization finish!）
  tb=$(grep -m1 "init begins" "$f" 2>/dev/null | sed 's/.*\[INFO\] \[\([0-9.]*\)\].*/\1/')
  tf=$(grep -m1 "Initialization finish" "$f" 2>/dev/null | sed 's/.*\[INFO\] \[\([0-9.]*\)\].*/\1/')
  win=$(awk -v a="$tb" -v b="$tf" 'BEGIN{ if (a!="" && b!="") printf "%.3f", b-a; else print "?" }')
  nun=$(grep -c "numerical unstable" "$f" 2>/dev/null)
  printf "  第 %d 次: %-42s  窗口 %ss  unstable %s\n" \
         "$i" "${line:-（20 秒没等到）}" "$win" "$nun"
  pkill -INT -f "[v]ins_node" 2>/dev/null
  sleep 3
done

echo "------------------------------------------------------------------"
echo "  读数已存 $LOG/bg_probe_*.log"
echo "  ★ 全部 SIGINT，没有 kill -9（硬杀会把相机搞卡）"
