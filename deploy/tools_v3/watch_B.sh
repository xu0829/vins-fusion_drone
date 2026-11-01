#!/bin/bash
# B 轮漂移采样器：每分钟把 pos_watch.csv 的最新一行压成一行摘要打出来。
#
# 为什么不直接 tail pos_watch.py 的输出：它走管道时是块缓冲的，管道不关就
# 什么都吐不出来（今晚踩过）。直接读 CSV 文件才是可靠的。
#
# pos_watch.csv 列（19 列，★ 别再猜列号，先读表头）：
#   t_wall,t_ros,vio_x,vio_y,vio_z,|vio|,|v_vio|,prop_x,prop_y,prop_z,|prop|,
#   pn_x,pn_y,pn_z,ad_x,ad_y,ad_z,acc_norm,gyro_norm
# 其中 |prop| 是【纯 IMU 积分】的位置模长，ad_* 是 att_delta（单位已是度）。
set +u
CSV=/tmp/vins_pi/pos_watch.csv
N=${1:-900}

echo "=== B 轮采样开始  $(date -u +%H:%M:%S) UTC ==="
printf "%-9s %12s %12s %8s %10s %9s %9s %8s\n" \
       时间 "|VIO|" "|纯IMU|" 差值比 "|v_VIO|" "att_roll" "att_pitch" "acc"

for i in $(seq 1 "$N"); do
  if [ -s "$CSV" ]; then
    tail -1 "$CSV" | awk -F, -v t="$(date -u +%H:%M:%S)" '
      NF >= 19 {
        vio=$6; prop=$11; v=$7; ax=$15; ay=$16; an=$18
        d = (prop>1e-9) ? (vio-prop)/prop*100 : 0
        printf "%-9s %12.2f %12.2f %7.1f%% %10.3f %9.2f %9.2f %8.2f\n",
               t, vio, prop, d, v, ax, ay, an
      }'
  fi
  sleep 60
done
echo "=== B 轮采样结束  $(date -u +%H:%M:%S) UTC ==="
echo "CSV 行数：$(wc -l < "$CSV" 2>/dev/null || echo 0)"
