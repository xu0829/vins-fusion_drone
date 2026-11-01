#!/bin/bash
# B 轮守卫：**只在出事时说话**，安静时一行都不打（Monitor 把每行变成一条通知）。
#
# 为什么另写一个而不用 pos_watch：pos_watch.py 是 open(path,"w")，
# 重启它会**截断已经录好的 CSV**。本脚本【只读】，不写任何数据文件。
#
# 事件格式：  时间 | 类型 | 说明
#   DRIFT  |VIO| 相对上次上报值变化 > 1 米（跑飞，或又被 camera wedge 注入）
#   WARN   numerical unstable / feature not enough 计数增加
#   DEAD   vins_node 或 relay 不在了
#   CAM    相机日志新增 watchdog / MIPI / 重新上线（wedge 前兆）
#   STOP   CSV 不再更新（记录器死了或被 timeout 掐了）
#
# 用法（容器内）：bash b_guard.sh [循环次数]        # 每次 15 秒，默认 240 → 1 小时
set +u
CSV=${CSV:-/tmp/vins_pi/pos_watch_B2.csv}
VLOG=${VLOG:-/tmp/vins_pi/vins.log}
CLOG=${CLOG:-/tmp/vins_pi/camera.log}
N=${1:-240}
THRESH=${THRESH:-1.0}        # 漂移阈值（米）

# 起始基线：取当前值，否则第一条消息就是假警报
last=""
if [ -s "$CSV" ]; then
  last=$(tail -1 "$CSV" | awk -F, 'NF>=6 {print $6}')
fi
cnt() { grep -c "$1" "$2" 2>/dev/null | head -1; }   # ★ grep -c 无匹配时返回 1，别接 &&
pw=$(cnt 'numerical unstable' "$VLOG"); pw=${pw:-0}
pf=$(cnt 'not enough' "$VLOG");          pf=${pf:-0}
pc=$(cnt 'watchdog triggered' "$CLOG");  pc=${pc:-0}
pu=$(cnt 'RealSense Node Is Up' "$CLOG"); pu=${pu:-0}
stale=0

for i in $(seq 1 "$N"); do
  sleep 15
  ts=$(date -u +%H:%M:%S)

  pgrep -f "[v]ins_node" >/dev/null 2>&1 || { echo "$ts | DEAD | vins_node 不在了"; break; }
  pgrep -f "[v]ins_diag_relay" >/dev/null 2>&1 || { echo "$ts | DEAD | vins_diag_relay 不在了"; break; }

  w=$(cnt 'numerical unstable' "$VLOG"); w=${w:-0}
  [ "$w" -gt "$pw" ] && { echo "$ts | WARN | numerical unstable $pw → $w"; pw=$w; }
  f=$(cnt 'not enough' "$VLOG"); f=${f:-0}
  [ "$f" -gt "$pf" ] && { echo "$ts | WARN | feature not enough $pf → $f"; pf=$f; }

  c=$(cnt 'watchdog triggered' "$CLOG"); c=${c:-0}
  [ "$c" -gt "$pc" ] && { echo "$ts | CAM | uvc watchdog $pc → $c（第 $c 次 wedge）"; pc=$c; }
  u=$(cnt 'RealSense Node Is Up' "$CLOG"); u=${u:-0}
  [ "$u" -gt "$pu" ] && { echo "$ts | CAM | 相机重新上线 $pu → $u"; pu=$u; }

  # CSV 是否还在长（mtime 超过 60 秒没动 = 记录器死了）
  age=$(( $(date +%s) - $(stat -c %Y "$CSV" 2>/dev/null || echo 0) ))
  if [ "$age" -gt 60 ]; then
    stale=$((stale+1))
    [ "$stale" -eq 1 ] && echo "$ts | STOP | $CSV 已 $age 秒没更新（记录器死了？）"
  else
    stale=0
  fi

  cur=$(tail -1 "$CSV" 2>/dev/null | awk -F, 'NF>=6 {print $6}')
  [ -z "$cur" ] && continue
  if [ -n "$last" ]; then
    big=$(awk -v a="$cur" -v b="$last" -v t="$THRESH" \
          'BEGIN{d=a-b; if(d<0)d=-d; print (d>t)?1:0}')
    if [ "$big" = "1" ]; then
      echo "$ts | DRIFT | |VIO| $last → $cur 米（Δ $(awk -v a="$cur" -v b="$last" 'BEGIN{printf "%.3f", a-b}')）"
      last=$cur
    fi
  fi
done
