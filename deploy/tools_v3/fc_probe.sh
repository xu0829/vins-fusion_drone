#!/bin/bash
# 飞控到底活没活 —— 专门用来查 fc_ping.py 这个探针本身可不可信。
#
# 背景：B 轮启动器在第 0 步判「飞控没心跳」并拒绝启动，但同一时刻
#       /imu 正以 195 Hz 在发。所以要么飞控真的断过，要么【探针在骗人】。
#       两条都要有独立证据，不能只信一个探针。
#
# ★ 本脚本刻意不写 set -u（ROS 的 profile.d 引用未定义的 ROS_MASTER_URI）
set +u
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

echo "=== 1. 谁在占着串口 / 谁在跑 ==="
pgrep -af "px4_imu_node" | grep -v "pgrep" || echo "  px4_imu_node 不在跑"
pgrep -af "wait_fc_then_start" | grep -v pgrep || echo "  wait_fc_then_start 不在跑"

echo
echo "=== 2. 设备节点 + 它在 USB 上的哪个口 ==="
ls -l /dev/ttyACM* 2>/dev/null || echo "  ★ /dev/ttyACM* 不存在"
for d in /sys/bus/usb/devices/*/; do
  v=$(cat "$d/idVendor" 2>/dev/null)
  case "$v" in
    1b8c|3162|26ac)
      echo "  $d  $v:$(cat $d/idProduct 2>/dev/null)  speed=$(cat $d/speed 2>/dev/null)  devnum=$(cat $d/devnum 2>/dev/null)"
      ;;
  esac
done

echo
echo "=== 3. /imu 是活的还是陈货（★ 转发层会在源死之后继续发）==="
echo -n "  最新一帧的 header.stamp: "
timeout 10 rostopic echo -n 1 /imu/header/stamp 2>/dev/null | tr '\n' ' '
echo
python3 - <<'PY'
import rospy, time
rospy.init_node('fc_probe', anonymous=True, disable_signals=True)
now = rospy.Time.now().to_sec()
got = []
def cb(m):
    got.append(m.header.stamp.to_sec())
rospy.Subscriber('/imu', rospy.AnyMsg, cb) if False else None
import sensor_msgs.msg as sm
rospy.Subscriber('/imu', sm.Imu, cb)
t0 = time.time()
while time.time() - t0 < 8 and len(got) < 5:
    time.sleep(0.1)
if not got:
    print("  ★ 8 秒内一帧 /imu 都没收到 —— 桥没在发")
else:
    print("  收到 %d 帧；最新帧时间戳 %.3f，ros 当前 %.3f，年龄 %.3f 秒"
          % (len(got), got[-1], now, now - got[-1]))
    print("  ⇒ 年龄 < 1 秒 = 真活；年龄很大 = 陈货")
PY

echo
echo "=== 4. fc_ping.py 完整输出（保留退出码和 stderr）==="
timeout 45 python3 /root/xu_drone/live/fc_ping.py
echo "  [fc_ping 退出码 $?]"

echo
echo "=== 5. fc_ping.py 怎么写的（看它等多久、开哪个口）==="
grep -nE "port|PORT|ttyACM|timeout|baud|except|return|sys.exit" /root/xu_drone/live/fc_ping.py | head -20
