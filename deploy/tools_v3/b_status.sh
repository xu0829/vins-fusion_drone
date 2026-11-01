#!/bin/bash
# B 轮健康速查：一条命令看全 —— 免得又踩 ssh→docker→bash 的嵌套引号（$2 会被吃掉）。
set +u
L=/tmp/vins_pi
echo "=== 进程 ==="
pgrep -af "[v]ins_node"    | head -1 || echo "  ✗ vins_node 不在跑"
pgrep -af "[v]ins_diag_relay" | head -1 || echo "  ✗ relay 不在跑"
pgrep -af "[p]x4_imu_node" | head -1 || echo "  ✗ IMU 桥不在跑"
pgrep -af "[e]mitter_guard" | head -1 || echo "  ✗ 守卫不在跑"

echo
echo "=== vins.log ==="
echo "  大小 $(wc -c < $L/vins.log 2>/dev/null || echo 0) 字节，最后写入 $(date -u -r $L/vins.log +%H:%M:%S 2>/dev/null || echo 未知) UTC（现在 $(date -u +%H:%M:%S)）"
echo "  numerical unstable : $(grep -c 'numerical unstable' $L/vins.log 2>/dev/null || echo 0)"
echo "  feature not enough : $(grep -c 'not enough' $L/vins.log 2>/dev/null || echo 0)"
echo "  ★ 初始化解出的零偏  : $(grep -m1 'gyroscope bias initial calibration' $L/vins.log 2>/dev/null | sed 's/.*calibration *//; s/\x1b\[[0-9;]*m//g' || echo '（还没打印）')"

echo
echo "=== pos_watch.csv ==="
echo "  行数 $(( $(wc -l < $L/pos_watch.csv 2>/dev/null || echo 1) - 1 ))"
head -1 $L/pos_watch.csv 2>/dev/null
tail -4 $L/pos_watch.csv 2>/dev/null
