#!/bin/bash
# 容器内运行：等飞控回来，再把 IMU 桥接拉起来。
#
# 为什么需要这个外部监督，而不是直接挂着 px4_imu_node.py 等它自愈：
#   它的重连分支写作 `if self.master is None and not self.reconnect(...)`，
#   但 connect() 失败时 self.master 已经被赋成一个死连接对象（第 79 行先赋值，
#   第 81 行 wait_heartbeat 才失败抛异常）。于是 master 永远不为 None，
#   重连分支此后再不进入 —— 它只是攥着旧 fd 空转 recv_match，不会自愈。
#
#   所以：起节点前必须先确认飞控是活的。这个脚本负责那一步。
#
# 用法（容器内）:
#   nohup bash /root/vinsfusion/tools/wait_fc_then_start_imu.sh > /dev/null 2>&1 &
set -u

PORT=${1:-/dev/ttyACM0}
TOOLS=/root/vinsfusion/tools
LOG=/tmp/imu_supervisor.log

# 必须 source，否则拉起的节点会以 ModuleNotFoundError: No module named 'rospy' 当场死掉
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

echo "[$(date '+%F %T')] 监督启动，等待飞控心跳（端口 $PORT）" >> "$LOG"

while true; do
    if timeout 25 python3 "$TOOLS/fc_ping.py" "$PORT" >> "$LOG" 2>&1; then
        echo "[$(date '+%F %T')] 飞控回来了，启动 IMU 桥接" >> "$LOG"
        cd "$TOOLS" || exit 1
        IMU_CSV=/tmp/imu_raw_recover.csv nohup python3 -u px4_imu_node.py "$PORT" \
            >> /tmp/imu_node_recover.log 2>&1 &
        echo "[$(date '+%F %T')] 已拉起 px4_imu_node，PID $!" >> "$LOG"
        exit 0
    fi
    echo "[$(date '+%F %T')] 还没心跳，15 秒后再试" >> "$LOG"
    sleep 15
done
