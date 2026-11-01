#!/bin/bash
# 【容器内运行】起 Foxglove 桥 + IMU 姿态节点，用来亲眼看 IMU 飘不飘。
#
# 为什么要有这个脚本：PX4 的 HIGHRES_IMU 不含姿态，Foxglove 里直接看 /imu
# 只有原始 xyz，看不到 roll/pitch/yaw。imu_rpy_node.py 把它算出来，并且
# 故意分成三路（纯陀螺 / 纯加计 / 互补）—— 摆一起看就是最直接的判据。
#
# 用法（在容器里）:
#   bash /root/vinsfusion/tools/start_imu_viz.sh
#
# 然后在【宿主】上另开一个终端:
#   bash ~/xuqixuan_learning/vinsfusion/tools/start_foxglove_vins.sh
# 注意：这里【不能】用 set -u。ROS 的 setup.bash 会 source
# /opt/ros/noetic/etc/catkin/profile.d/10.roslaunch.sh，里面引用了
# 尚未定义的 ROS_MASTER_URI，开了 -u 会直接报 "unbound variable" 退出。
source /opt/ros/noetic/setup.bash

PORT=${PORT:-8767}
TOOLS=$(cd "$(dirname "$0")" && pwd)

echo "== 1/3 roscore =="
if pgrep -f "[r]oscore" >/dev/null; then
  echo "   已在跑"
else
  nohup roscore > /tmp/roscore_imu.log 2>&1 &
  sleep 4
  echo "   已启动"
fi

echo "== 2/3 foxglove_bridge (端口 $PORT) =="
if pgrep -f "[f]oxglove_bridge" >/dev/null; then
  echo "   已在跑"
else
  nohup rosrun foxglove_bridge foxglove_bridge _port:=$PORT _address:=0.0.0.0 \
      > /tmp/foxglove_imu.log 2>&1 &
  sleep 3
  echo "   已启动"
fi

echo "== 3/3 imu_rpy 节点 =="
if pgrep -f "[i]mu_rpy_node.py" >/dev/null; then
  echo "   已在跑"
else
  nohup python3 -u "$TOOLS/imu_rpy_node.py" > /tmp/imu_rpy.log 2>&1 &
  sleep 2
  echo "   已启动"
fi

echo
echo "== 检查 =="
echo "-- /imu 频率 --"
timeout 8 rostopic hz /imu 2>&1 | grep -E "average|no new" | head -2
echo "-- 姿态话题 --"
for t in /imu/rpy_deg /imu/att_gyro_deg /imu/att_accel_deg /imu/orientation; do
  if timeout 3 rostopic list 2>/dev/null | grep -qx "$t"; then
    echo "   ✅ $t"
  else
    echo "   ❌ $t  (没发出来 —— 看 /tmp/imu_rpy.log)"
  fi
done

echo
echo "=========================================================="
echo "在【宿主】另开终端执行:"
echo "  bash ~/xuqixuan_learning/vinsfusion/tools/start_foxglove_vins.sh"
echo
echo "Foxglove 里手动填连接地址:  ws://127.0.0.1:$PORT"
echo "布局文件（导入）: ~/xuqixuan_learning/vinsfusion/tools/foxglove_imu_layout.json"
echo "=========================================================="
echo
echo "怎么判读（设备静放别动，看 1~2 分钟）:"
echo "  Plot!roll / Plot!pitch 里三条线:"
echo "    加计(不累积误差) = 参考线，看它平不平"
echo "    陀螺积分(会飘)   = 匀速偏离参考线 -> 陀螺有零偏"
echo "    互补             = 实际能用的姿态"
echo "  偏离速率反推零偏: 每秒偏 0.01° 约等于 36°/h 的零偏"
echo
echo "  原始 xyz 在 Plot!gyro / Plot!accel:"
echo "    静止时陀螺 xyz 应当【围绕 0 抖】，均值不偏 = 零偏已标掉"
echo "    加计静止时模长应 ≈ 9.8"
echo
echo "  yaw 必然缓慢漂 —— 没有磁力计，无绝对航向参考，这是物理限制"
echo
echo "实时读数（不开 Foxglove 也能看）:"
echo "  tail -f /tmp/imu_rpy.log"
