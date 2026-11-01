#!/bin/bash
# 宿主侧：启动 Foxglove 连本机 Docker 里的 VINS-Fusion（实时看漂移）。
#
# 复用你现有那套 Foxglove 3.2.1（/home/xu/drone/foxglove/app/...），只是换个连接地址。
#
# 端口分配（避免和现有两套打架）：
#   8765 = kswlt
#   8766 = d430_view（香橙派）
#   8767 = 本机 Docker 里的 VINS-Fusion  ← 本脚本
#
# 用法:
#   bash tools/start_foxglove_vins.sh            # 连本机 8767
#   PORT=8765 bash tools/start_foxglove_vins.sh  # 临时换端口
set -u

APP=/home/xu/drone/foxglove/app/opt/Foxglove/foxglove-studio
PORT=${PORT:-8767}
HOST=127.0.0.1
LAYOUT=${LAYOUT:-/home/xu/xuqixuan_learning/vinsfusion/tools/foxglove_vins_layout.json}

if [ ! -x "$APP" ]; then
  echo "找不到 Foxglove 程序: $APP"
  echo "（你现有的那套在 /home/xu/drone/foxglove/，如果换过位置请改本脚本里的 APP）"
  exit 1
fi

echo "== 检查桥端口 $HOST:$PORT =="
if timeout 2 bash -c "echo > /dev/tcp/$HOST/$PORT" 2>/dev/null; then
  echo "端口已就绪。"
else
  echo "端口没在听 —— 容器里的 foxglove_bridge 还没起。"
  echo "先在【容器】里执行:"
  echo "  docker exec -it vins_noetic bash"
  echo "  bash /root/vinsfusion/tools/start_viz.sh <配置> <bag>"
  echo
  echo "（本脚本仍会启动 Foxglove，你可以先导入布局、等桥起来再连。）"
fi

echo
echo "布局: $LAYOUT"
echo "  导入方式：布局 → 导入布局 → 选上面这个文件"
echo "  手动连接地址: ws://$HOST:$PORT"
echo

if pgrep -f "$APP" >/dev/null; then
  echo "Foxglove 已在运行，在窗口里「打开连接」填 ws://$HOST:$PORT 即可。"
  exit 0
fi

echo "== 启动 Foxglove =="
exec "$APP" --no-sandbox --lang=zh-CN "$@"
