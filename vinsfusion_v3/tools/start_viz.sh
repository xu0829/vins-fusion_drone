#!/usr/bin/env bash
# 一边回放 bag 跑 VINS，一边开 foxglove_bridge，供宿主上的 Foxglove 实时看漂移。
#
# 为什么用 foxglove_bridge 而不是 rosbridge：
#   1. 只要 3 个包（rosbridge 要 41 个），而磁盘只剩 4.1 G；
#   2. 你香橙派上那套 d430_view 用的就是 foxglove_bridge，
#      Foxglove Studio 3.2.1 连它没有任何握手问题，换 rosbridge 反而要重新趟协议。
#   3. 容器本来就是 host 网络，浏览器/桌面端直接连 127.0.0.1 就行。
#
# 为什么不在容器里跑 rviz：
#   容器没挂 X11 socket、也没直通 GPU，跑 rviz 要在宿主架 TCP→unix 转发 + 改 xhost，
#   还得装 111 个包（约 600 MB）。foxglove_bridge 这条路不碰这些。
#
# 用法（容器内，已 source /opt/ros/noetic/setup.bash）：
#   bash tools/start_viz.sh <配置> <bag> [速率]
#
# 然后宿主机上（另开一个终端）：
#   bash ~/xuqixuan_learning/vinsfusion/tools/start_foxglove_vins.sh
#   在里面「打开连接」-> ws://127.0.0.1:8767
#   布局 -> 导入布局 -> tools/foxglove_vins_layout.json
set -u

PORT=${PORT:-8767}

if [[ -f /root/catkin_ws/devel/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /root/catkin_ws/devel/setup.bash
fi
if ! rospack find vins >/dev/null 2>&1; then
  echo "找不到 'vins' 包，先确认已编译并 source /root/catkin_ws/devel/setup.bash"
  exit 1
fi
if ! rospack find foxglove_bridge >/dev/null 2>&1; then
  echo "没装 foxglove_bridge。先执行："
  echo "  apt-get install -y ros-noetic-foxglove-bridge"
  exit 1
fi

CONFIG="${1:?需要主配置路径}"
BAG="${2:?需要 bag 路径}"
RATE="${3:-1.0}"

# --- roscore ---
if ! rostopic list >/dev/null 2>&1; then
  echo "[1/4] 启动 roscore"
  setsid nohup roscore > /tmp/roscore.log 2>&1 < /dev/null &
  for _ in $(seq 1 30); do
    rostopic list >/dev/null 2>&1 && break
    sleep 0.5
  done
fi

# --- foxglove_bridge ---
if rostopic list 2>/dev/null | grep -q . && timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
  echo "[2/4] foxglove_bridge 已在 $PORT 上"
else
  echo "[2/4] 启动 foxglove_bridge（端口 $PORT）"
  setsid nohup rosrun foxglove_bridge foxglove_bridge \
    _port:="$PORT" _address:=0.0.0.0 > /tmp/foxglove_bridge.log 2>&1 < /dev/null &
  for _ in $(seq 1 20); do
    timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null && break
    sleep 0.5
  done
  if timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
    echo "      桥已就绪"
  else
    echo "      桥没起来，看 /tmp/foxglove_bridge.log"; tail -10 /tmp/foxglove_bridge.log
  fi
fi

# --- vins_node ---
echo "[3/4] 启动 vins_node（配置 $CONFIG）"
setsid nohup rosrun vins vins_node "$CONFIG" > /tmp/vins_live.log 2>&1 < /dev/null &
VINS_PID=$!
sleep 4
kill -0 "$VINS_PID" 2>/dev/null || { echo "vins_node 立即退出："; tail -20 /tmp/vins_live.log; exit 1; }

cat <<EOF

=====================================================================
  桥已经在 127.0.0.1:$PORT 上听。现在去宿主机【另开一个终端】执行：

     bash ~/xuqixuan_learning/vinsfusion/tools/start_foxglove_vins.sh

  然后在 Foxglove 里：
     1. 打开连接 -> ws://127.0.0.1:$PORT
     2. 布局 -> 导入布局 -> tools/foxglove_vins_layout.json

  3D 面板里把 Topic 设成 /vins_estimator/path，就能看着轨迹一条条画出去
  —— 那就是漂移的样子。右边的 Plot 面板直接画 x/y/z 随时间的曲线。

  注意：bag 还没开始播。确认 Foxglove 连上、布局导好了，再回这里回车。
=====================================================================

EOF
read -r -p "按回车开始回放 bag（$BAG，速率 $RATE）..."

echo "[4/4] 回放 bag"
rosbag play -r "$RATE" "$BAG"
echo "      回放结束。桥和 vins_node 还开着，可以继续在 Foxglove 里翻看。"
echo "      收工： pkill -f foxglove_bridge; pkill -f vins_node"
