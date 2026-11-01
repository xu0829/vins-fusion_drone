#!/bin/bash
# drift_watch 的启停壳（容器内用）。
#   bash drift_watch.sh start [间隔秒]   # 默认 2 秒；已在跑则不动
#   bash drift_watch.sh stop
#   bash drift_watch.sh status
#   bash drift_watch.sh tail [行数]
#
# 日志: /tmp/vins_pi/drift_watch.log   CSV: /tmp/vins_pi/drift_watch.csv
# 停止用 SIGINT（和本仓库其它脚本一致，不用 kill -9）。
set +u
TOOLS=/root/xu_drone/live
LOG=/tmp/vins_pi/drift_watch.log
PIDF=/tmp/vins_pi/drift_watch.pid

case "$1" in
  start)
    if pgrep -f "[d]rift_watch.py" >/dev/null 2>&1; then
      echo "已经在跑（pid $(pgrep -f '[d]rift_watch.py' | tr '\n' ' ')）"; exit 0
    fi
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash
    setsid nohup python3 -u "$TOOLS/drift_watch.py" "${2:-2}" 0 >> "$LOG" 2>&1 &
    echo $! > "$PIDF"
    sleep 4
    if pgrep -f "[d]rift_watch.py" >/dev/null 2>&1; then
      echo "✓ 已启动（pid $(cat $PIDF)，日志 $LOG）"
      tail -3 "$LOG"
    else
      echo "✗ 没起来，看 $LOG："; tail -10 "$LOG"; exit 1
    fi
    ;;
  stop)
    pkill -INT -f "[d]rift_watch.py" && echo "已发 SIGINT" || echo "没在跑"
    ;;
  status)
    if pgrep -f "[d]rift_watch.py" >/dev/null 2>&1; then
      echo "运行中: pid $(pgrep -f '[d]rift_watch.py' | tr '\n' ' ')"
      echo "CSV 行数: $(wc -l < /tmp/vins_pi/drift_watch.csv 2>/dev/null || echo 0)"
      tail -5 "$LOG" 2>/dev/null
    else
      echo "没在跑"
    fi
    ;;
  tail)
    tail -"${2:-40}" "$LOG"
    ;;
  *)
    sed -n '2,9p' "$0"; exit 1
    ;;
esac
