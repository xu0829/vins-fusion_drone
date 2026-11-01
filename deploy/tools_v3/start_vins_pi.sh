#!/bin/bash
# 香橙派容器 vins_pi 上的实时全栈启动器。
#
# 与笔记本 start_vins_drone.sh 的关系：同一个 7 步骨架，三处按今晚的教训改了。
#
# 与香橙派旧脚本 start_vins_live.sh 的区别（旧脚本一字未动，留在原地可回滚）：
#   1) ★ 发射器守卫与相机【同时】启动。旧脚本是"等相机起来 → sleep 40 → 关一次"，
#      而 librealsense 在启动时会自己发布 emitter_enabled=1，那个初始化窗口
#      老写法结构性地覆盖不到（笔记本上抓到两次：12:07:20、12:16:43）。
#      这里改成相机 roslaunch 的同时就把 emitter_guard.py 挂上，它内部会等
#      /camera/stereo_module/set_parameters 出现（最多 180 秒），所以不会抢跑。
#   2) 加了预检：飞控心跳（USB 枚举着 ≠ 固件在跑；飞控每次上电会走
#      bootloader→固件两次枚举，/dev/ttyACM0 一直都在，不能拿它当判据）
#      + 相机读 sysfs 找 8086:0ad4（不看 /dev/video*：librealsense 走 libusb
#      直通，启动时会把 uvcvideo 摘掉，那是正常行为，不是掉线）。
#   3) 加了后置确认：/imu 有没有数据、vins_node 还在不在、/diag/* 逐条验活。
#      今晚被"转发层在源死之后继续发"骗过，所以每条都要单独确认。
#
# 顺序：预检 → roscore → 相机 + 发射器守卫 → 关发射器 → 等飞控心跳起 IMU 桥
#       → foxglove_bridge → vins_node → 诊断转发 → 复查发射器
#
# 用法（容器内）：
#   bash /root/start_vins_pi.sh
# 前台跑，Ctrl-C 优雅停止。全程 SIGINT，绝不用 kill -9 —— 硬杀会把相机搞卡，
# 只能靠 USB 授权位复位救。
#
# 必须在 source 之前先给 ROS_MASTER_URI/ROS_HOSTNAME 赋值：
# catkin 的 profile.d/10.roslaunch.sh 会直接读 $ROS_MASTER_URI，
# 配合下面的 set -u 就变成 "unbound variable" 当场退出（踩过）。
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
export ROS_IP=${ROS_IP:-127.0.0.1}

set -u

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
[ -f /root/catkin_ws/devel/setup.bash ] && source /root/catkin_ws/devel/setup.bash

LOG=/tmp/vins_pi; mkdir -p "$LOG"
TOOLS=/root/xu_drone/live
REPO=/root/catkin_ws/src/VINS-Fusion
PORT=${PORT:-8767}                       # 8766 在香橙派上归 d430_view，别撞
FC_PORT=${FC_PORT:-/dev/ttyACM0}
CONFIG=${CONFIG:-$REPO/config/d430_px4/d430_px4_live_refnoise.yaml}
ENABLE_FOXGLOVE=${ENABLE_FOXGLOVE:-1}  # 0 = 不启动 foxglove_bridge（不往笔记本传数据）

mkdir -p /root/calib_data/vins_output/pose_graph

cleanup() {
  echo
  echo "[launcher] 收到停止信号，优雅关闭中（全程 SIGINT，不用 kill -9）..."
  for pat in emitter_guard vins_diag_relay vins_node px4_imu_node wait_fc_then_start foxglove_bridge; do
    pkill -INT -f "$pat" 2>/dev/null
  done
  pkill -INT -f "roslaunch realsense2_camera" 2>/dev/null
  sleep 10
  pkill -INT -f roscore 2>/dev/null
  sleep 3
  echo "[launcher] 关闭完成"
  exit 0
}
trap cleanup SIGTERM SIGINT

# 有数据到达就返回 0（比 rostopic hz 快，hz 要攒够样本才报速率）
has_data() { timeout 4 rostopic echo -n 1 "$1" >/dev/null 2>&1; }

# 读当前发射器状态。get_parameters 服务不存在，只能读这个话题。
emitter_state() {
  timeout 5 rostopic echo -n 1 /camera/stereo_module/parameter_updates 2>/dev/null \
    | grep -A1 '"emitter_enabled"' | tail -1 | tr -d ' ' | sed 's/^value://'
}
emitter_off() {
  rosrun dynamic_reconfigure dynparam set /camera/stereo_module emitter_enabled 0 >/dev/null 2>&1
}

# 清掉上一轮残留的本栈进程。★ 2026-09-28 加：原来这份名单只写在 cleanup() 里，
# 只在【停止】时跑，于是上一次会话留下的 px4_imu_node 会活到下一次启动，
# 造成两个后果：
#   1) step 0 的 fc_ping 预检要再开【第二个】读者读 /dev/ttyACM0，两个读者把
#      CDC 字节流劈开，心跳包被劈成两半 -> 间歇性误报"飞控没心跳"并拒绝启动
#      （B 轮因此连挂两次，实测同一 pid、同样条件下一次过一次不过）
#   2) step 4 自己还要再拉一个 px4_imu_node，两个抢同一个串口，两边都半死不活
# ⇒ 名单从 cleanup() 提到启动路径上，两处共用。
# ★ 故意【不】杀：roscore / 相机 roslaunch（杀了就要重来一次 initial_reset，
#   多一次 USB 风险，而且曝光要重新收敛）/ emitter_guard（少一个守卫的
#   空窗期没有好处；多一个守卫只是重复把 emitter 按到 0，无害）。
stop_stale_stack() {
  local pat
  for pat in "[v]ins_node" "[v]ins_diag_relay" "[f]oxglove_bridge" \
             "[p]x4_imu_node" "[w]ait_fc_then_start_imu"; do
    pkill -INT -f "$pat" 2>/dev/null && echo "  已通知残留进程退出：$pat"
  done
  # 等它们真的走干净（px4_imu_node 要松开 /dev/ttyACM0 才轮得到预检）
  local i
  for i in $(seq 1 15); do
    pgrep -f "[p]x4_imu_node" >/dev/null 2>&1 || break
    sleep 1
  done
  return 0
}

# 相机栈是不是还有进程活着（roslaunch / nodelet manager / nodelet load 三者之一）。
# ★ 为什么必须有这个函数：判"相机在不在"只看话题是不够的，见 2/7 步的长注释。
cam_stack_alive() {
  pgrep -f "[r]oslaunch realsense2_camera" >/dev/null 2>&1 && return 0
  pgrep -f "[n]odelet (manager|load)"       >/dev/null 2>&1 && return 0
  return 1
}

echo "================================================================"
echo "[launcher] 0a/7 清理上一轮残留进程"
stop_stale_stack
if pgrep -f "[p]x4_imu_node" >/dev/null 2>&1; then
  echo "  ★ px4_imu_node 还活着，/dev/ttyACM0 仍被占；预检很可能误报"
else
  echo "  ✓ 串口没有别的读者了"
fi
echo "================================================================"
echo "[launcher] 0/7 预检"
if ! timeout 25 python3 "$TOOLS/fc_ping.py" "$FC_PORT" > "$LOG/fc_ping.log" 2>&1; then
  echo "  ✗ 飞控没心跳（$FC_PORT）"
  echo "    确认飞控插好、通电、不是只插了 USB 没上电。探针输出："
  sed 's/^/    /' "$LOG/fc_ping.log" | tail -4
  exit 1
fi
echo "  ✓ 飞控有心跳（$FC_PORT）"

# 相机预检：读 sysfs 找 8086:0ad4。不用 lsusb（容器里没装 usbutils），
# 也不看 /dev/video*（librealsense 走 libusb，那个判据会假报）。
CAM_PATH=""
for f in /sys/bus/usb/devices/*/idVendor; do
  [ -r "$f" ] || continue
  if [ "$(cat "$f" 2>/dev/null)" = "8086" ] && [ "$(cat "${f%idVendor}idProduct" 2>/dev/null)" = "0ad4" ]; then
    CAM_PATH="$(dirname "$f")"; break
  fi
done
if [ -z "$CAM_PATH" ]; then
  echo "  ✗ sysfs 里找不到 D430（8086:0ad4）—— 相机不在 USB 总线上"
  echo "    当前可见 USB 设备：$(ls -d /sys/bus/usb/devices/*/idVendor 2>/dev/null | wc -l) 个"
  exit 1
fi
echo "  ✓ 相机在总线上（$CAM_PATH，$(cat $CAM_PATH/speed 2>/dev/null) Mbps）"

if [ ! -f "$CONFIG" ]; then
  echo "  ✗ 配置不存在：$CONFIG"; exit 1
fi
echo "  ✓ 配置 $CONFIG"

echo "[launcher] 1/7 roscore"
if ! rostopic list >/dev/null 2>&1; then
  roscore > "$LOG/roscore.log" 2>&1 &
  for _ in $(seq 1 30); do
    rostopic list >/dev/null 2>&1 && break
    sleep 0.5
  done
fi
if ! rostopic list >/dev/null 2>&1; then
  echo "  ✗ roscore 起不来，看 $LOG/roscore.log"; exit 1
fi
echo "  ✓ roscore"

echo "[launcher] 2/7 相机 640x480x30（infra1+infra2）"
# ★ 复跑时必须先问一句"相机是不是已经在发了"。
#   闸门不过时是 `exit 1` 退出的（trap 只管信号），所以相机/roscore/守卫【是留着的】，
#   这时再无条件 roslaunch 一次，就会有【两个 realsense 节点抢同一个 USB 设备】——
#   两边都半死不活，而日志里只看到一堆 USB 报错，非常难查。
#   判据用话题本身（不是进程名、不是 /dev/video*）：rostopic echo 要收到真消息才算数，
#   僵尸注册（kill -9 之后话题还在）在这里会正确地判成"没数据"。
#
# ★★ 2026-10-06 补：上面那条判据【只对了一半】。实测到一种状态 —— 相机节点活着、
#    D430 被它占着、但码流没出来（话题注册在、一条消息都没有）。这时 has_data 判 false，
#    脚本就再 roslaunch 一次，正好踩中上面警告的那个坑。事后的 master.log 是铁证：
#      08:57:24,726  +PUB /camera/realsense2_camera_manager/bond   （新 manager 建 bond）
#      08:57:24,736  -PUB /camera/realsense2_camera_manager/bond   （10 毫秒后就被销毁）
#      08:57:34,8xx  Bond broken, exiting                          （正好 10.0 秒后）
#    新 nodelet 的 onInit 当场失败（老 nodelet 还攥着 USB 设备）⇒ manager 随即销毁 bond；
#    而 nodelet CLI 是【绑在 bond 上】的，收不到心跳就等满 10 秒，然后以【退出码 0】退出
#    ⇒ roslaunch 报 "process has finished cleanly"，既不报警也不重启，整个 roslaunch
#    跟着一起退。屏幕上看起来是"相机没起来"，真因是【新旧两个栈互相踩】。
#    ⇒ 规矩：同一时刻只允许一个相机栈。还有 realsense/nodelet 进程活着时，
#      要么证明它真的在出图（复用），要么先把它停干净、等 USB 设备释放，再起新的。
#
# ★ 反向证据（说明"管理器同名"本身不是杀手，占着设备的那个 nodelet 才是）：
#   08:44:09 起 5062 那轮时，上一轮 3870 的 manager 也还活着（只是里面没 nodelet，
#   它的 nodelet 早在 08:32:31 就 bond 超时退出了）—— 那一次相机【起来了】。
if has_data /camera/infra1/image_rect_raw; then
  CAM_ALREADY=1
  echo "  ✓ 相机已在发（复跑，不重启）—— 顺带省掉一次 initial_reset 的 USB 风险"
else
  CAM_ALREADY=0
  if cam_stack_alive; then
    echo "  ⚠ 相机进程还在，但图像没在发 —— 先优雅停掉它再起（绝不起第二个）"
    pkill -INT -f "[r]oslaunch realsense2_camera" 2>/dev/null
    pkill -INT -f "[n]odelet (manager|load)"      2>/dev/null
    for _ in $(seq 1 30); do
      cam_stack_alive || break
      sleep 1
    done
    if cam_stack_alive; then
      echo "  ✗ 等了 30 秒还没停干净，不硬杀（硬杀会把相机卡在 USB 上，只能靠重新插拔救）："
      pgrep -af "[r]oslaunch realsense2_camera|[n]odelet (manager|load)" | sed 's/^/      /'
      echo "    ⇒ 先收尾：bash $TOOLS/stop_vins_pi.sh   然后再重跑本脚本"
      exit 1
    fi
    sleep 2
    echo "  ✓ 旧相机栈已停干净（nodelet 关了设备、进程也没了）"
  fi
  # 上一轮的 roslaunch 可能还攥着 camera.log 的 fd：文件被 `>` 截断之后它再写就是
  # 从旧偏移量落笔，中间留一整片 NUL，grep 会报 "Binary file matches"。
  # 先把它转走，本轮新起一个干净的。
  [ -f "$LOG/camera.log" ] && mv -f "$LOG/camera.log" "$LOG/camera.log.prev"
  # ★ 2026-10-07：30 → 15。VINS 的优化吞吐只有 ~15Hz，而 sync_process 不丢帧，
  #   相机跑 30 会让 /vins_estimator/odometry 的时间戳落后无限增长（实测 35~65ms/s，
  #   已涨到 8 秒）。降到 15 是让输入速率落在 VINS 吃得住的范围内。
  #   回退：改回 30（备份 start_vins_pi.sh.bak-fps30-20261007）。
  roslaunch realsense2_camera rs_camera.launch \
    enable_infra1:=true enable_infra2:=true enable_color:=false enable_depth:=false \
    infra_width:=640 infra_height:=480 infra_fps:=30 initial_reset:=true \
    stereo_module/enable_auto_exposure:=false \
    stereo_module/exposure:=8500 \
    stereo_module/gain:=32 \
    > "$LOG/camera.log" 2>&1 &
fi

# ★★ 关键：守卫在【这一刻】就挂上，和相机并行。它内部会等 set_parameters 服务
#    出现（最多 180 秒），所以不会比相机早跑；而相机一就绪、librealsense 一发布
#    emitter_enabled=1，它就能当场关掉 —— 老写法"sleep 40 再关一次"漏掉的就是这一刻。
#    （相机是复用的也没关系：下结论只看 /camera/stereo_module/parameter_updates 的实测值。）
echo "[launcher] 2b/7 发射器守卫（与相机同时启动，日志 $LOG/emitter_guard.log）"
if pgrep -f "[e]mitter_guard" >/dev/null; then
  echo "  ✓ 守卫已在跑（复跑，不重复起）"
else
  python3 -u "$TOOLS/emitter_guard.py" > "$LOG/emitter_guard.log" 2>&1 &
fi

if [ "$CAM_ALREADY" = "1" ]; then
  echo "  ✓ 复用现有相机（判活直接看话题，不看日志 —— 日志是上一轮的了）"
else
  # ★ nodelet CLI 死了就别再等满 60 秒（2026-10-06 加）。
  #   nodelet 挂掉时 roslaunch 是【不会】退出的 —— manager 还在，它就一直在那等，
  #   屏幕上看不出来。真正先没命的是那个 `nodelet load` 进程，所以盯它：
  #   见过它、后来它没了 ⇒ 这一轮已经废了，立刻收场。
  seen_cli=0
  for _ in $(seq 1 60); do
    grep -q "RealSense Node Is Up" "$LOG/camera.log" 2>/dev/null && break
    if pgrep -f "[n]odelet load" >/dev/null 2>&1; then
      seen_cli=1
    elif [ "$seen_cli" = "1" ]; then
      echo "  ⚠ 相机 nodelet 进程已退出（roslaunch 不会重启它，bond 超时是 10 秒）"
      break
    fi
    sleep 1
  done
  if ! grep -q "RealSense Node Is Up" "$LOG/camera.log" 2>/dev/null; then
    echo "  ✗ 相机没起来，日志尾部："; sed 's/^/    /' "$LOG/camera.log" | tail -8; exit 1
  fi
  echo "  ✓ 相机就绪（等 8 秒让码流稳定）"
  sleep 8
fi
echo "[launcher] 2c/7 固定 IR 手动曝光"
MANUAL_EXPOSURE_OK=0
for i in $(seq 1 20); do
  rosrun dynamic_reconfigure dynparam set /camera/stereo_module \
    "{'enable_auto_exposure': False, 'exposure': 8500, 'gain': 32}" \
    >/dev/null 2>&1 || true

  CAMERA_CFG=$(rosrun dynamic_reconfigure dynparam get /camera/stereo_module 2>/dev/null || true)
  if printf '%s\n' "$CAMERA_CFG" | grep -q "'enable_auto_exposure': False" &&
     printf '%s\n' "$CAMERA_CFG" | grep -q "'exposure': 8500" &&
     printf '%s\n' "$CAMERA_CFG" | grep -q "'gain': 32"; then
    MANUAL_EXPOSURE_OK=1
    break
  fi
  sleep 1
done

[ "$MANUAL_EXPOSURE_OK" -eq 1 ] || {
  echo "✗ 相机未接受手动曝光，停止启动；不要继续启动 VINS"
  exit 1
}
echo "  ✓ 手动曝光：8500 us，gain 32，自动曝光已关"
echo "[launcher] 3/7 关 IR 发射器（不关会有散斑污染图像）"
emitter_off
sleep 2
EM=$(emitter_state)
echo "  ✓ 已发送 emitter_enabled=0   实测 ${EM:-（读不到，不代表没关）}"
echo "    守卫日志（应能看到它抓到过启动窗口里的 1）："
sed 's/^/      /' "$LOG/emitter_guard.log" | tail -4

# ---- 3.5/7 开测前的特征点数闸门 ------------------------------------------
# 为什么放在这里：这是"相机已就绪且发射器已关"、而 VINS 还没启动的唯一点。
# 放到启动后再看就是竞态，而且一旦不合格就得重启相机（initial_reset:=true），
# 每次重启都是一次 USB 风险 —— 能在 VINS 起来之前拦住，就别让它起来。
#
# 判据来自记忆 feature-starvation-diverges-vins：交流电、零掉线、IMU 196Hz 全干净，
# 照样 427 秒飞到 46.4km；开灯换视角后同一份代码 |VIO| 46.4km → 5.7 毫米。
# 基线：健康静止 83~108/150；饿死 63/32；发射器开着 42。
#
# ★ 阈值定在 80，别往下调。2026-09-28 香橙派第一轮我把它放松到 60 想让
#   "边界情况也能跑"，结果 73 被放行，那一轮 4 分钟飞到 5167 米。事后取证：
#   73 正落在记忆里已证实的饿死区间（60~75），而健康基线是 83~108。
#   放松阈值不是稳健，是把自己的判据调松到测不出问题。
GOOD=${GOOD:-80}
FEAT_MIN=${FEAT_MIN:-80}
echo "[launcher] 3.5/7 特征点数闸门（发射器已关，这是 VINS 看到画面的真实条件）"
# ★ 先让自动曝光稳住再量。2026-09-28 实测：关发射器/相机就绪之后，infra2 的
#   自动曝光还要约 10 秒才收敛（亮度 67→106、角点 48→100），在这期间量到的是
#   AE 过渡态，不是画面本身。上一轮就是拿 8 秒窗口里的过渡值判的卷。
sleep 6
FEAT_ALL=$(timeout 30 python3 "$TOOLS/feat_watch.py" 8 2>/dev/null)
echo "$FEAT_ALL" | sed 's/^/      /'
# 取窗口内的最大值，避免拿一个瞬时低谷做判决
N1=$(echo "$FEAT_ALL" | sed -n 's/.*infra1: *\([0-9]*\)\/.*/\1/p' | sort -n | tail -1)
N1=${N1:-0}
if [ "$N1" -lt "$FEAT_MIN" ]; then
  echo "  ✗ infra1 峰值 $N1/150 < $FEAT_MIN，视觉饿死。VINS 起来也只是纯 IMU 积分，必飘。"
  echo "    ⇒ 先【开灯】或把镜头转向有纹理的视角（别对着白墙/黑屏/空桌面），再重跑。"
  echo "    ⇒ 想先看画面长什么样：python3 $TOOLS/grab_frame.py /tmp/vins_pi"
  echo "      （存 infra1_now.png / infra2_now.png，能直接看出是不是对着空墙）"
  echo "    ⇒ 读数低不一定是"没纹理"：画面里一大块【过曝死白】也会压低它。"
  echo "      qualityLevel=0.01 是相对画面最强角点的，死白边缘把门限整体抬高，"
  echo "      中等强度的角点（书脊、木纹）就全被刷掉了。把强光源移出画面试试。"
  echo
  # ★ 故意【不】在这里清理：相机留着，用户才好边调视角边看 feat_watch 实时数字，
  #   不用为了看一眼就重启相机（60 秒 + 一次 USB 风险）。trap 只管信号，不管 exit，
  #   所以这里必须自己说清楚，否则会被误以为"什么都没在跑"。
  echo "  ┌────────────────────────────────────────────────────────────┐"
  echo "  │ 相机 / roscore / 发射器守卫【仍然开着】，这是故意的：        │"
  echo "  │ 调整视角时实时看角点数，不用重启相机：                       │"
  echo "  │   python3 $TOOLS/feat_watch.py 30                           │"
  echo "  │ 调好了直接重跑本脚本（它会认出现成的 roscore，不会重复起）。 │"
  echo "  │ 收尾：bash $TOOLS/stop_vins_pi.sh                            │"
  echo "  │       （本脚本已退出，pkill -f 本脚本名匹配不到东西，没用）  │"
  echo "  └────────────────────────────────────────────────────────────┘"
  exit 1
fi
if [ "$N1" -lt 100 ]; then
  echo "  ⚠ infra1 峰值 $N1/150 —— 够跑但偏薄，这个数字要记进本轮结论"
else
  echo "  ✓ infra1 峰值 $N1/150（健康）"
fi

echo "[launcher] 4/7 IMU 桥（外部监督：先确认飞控活着再拉 px4_imu_node）"
# ★ 同相机那条：两个 px4_imu_node 会抢同一个串口 /dev/ttyACM0，两边都半死不活。
if has_data /imu; then
  echo "  ✓ /imu 已在发（复跑，不重复起桥）"
else
  bash "$TOOLS/wait_fc_then_start_imu.sh" "$FC_PORT" > "$LOG/imu_super.log" 2>&1 &
  for _ in $(seq 1 60); do
    has_data /imu && break
    sleep 1
  done
fi
if ! has_data /imu; then
  echo "  ✗ /imu 没数据，看 $LOG/imu_super.log 和 /tmp/imu_node_recover.log"; exit 1
fi
echo "  ✓ /imu 在发"

echo "[launcher] 5/7 foxglove_bridge（端口 $PORT）"
if [ "$ENABLE_FOXGLOVE" != "1" ]; then
  # ★ 2026-10-11 加：开机自启默认关（systemd 里 ENABLE_FOXGLOVE=0）。没人在笔记本上看时，
  #   桥白占 CPU/带宽。关的时候顺手把残留的桥收掉 —— 否则"关"只是"不再新起一个"。
  #   现场要看图：宿主机 bash ~/vins_build/foxglove_on.sh，不必重启整个栈。
  if pkill -INT -f "[f]oxglove_bridge" 2>/dev/null; then
    echo "  ⚠ ENABLE_FOXGLOVE=$ENABLE_FOXGLOVE —— 不传数据，并已收掉残留的桥"
  else
    echo "  ⚠ ENABLE_FOXGLOVE=$ENABLE_FOXGLOVE —— 不传数据（本来也没在跑）"
  fi
else
  if timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
    echo "  端口已在听，跳过"
  else
    roslaunch foxglove_bridge foxglove_bridge.launch port:="$PORT" > "$LOG/foxglove.log" 2>&1 &
    for _ in $(seq 1 30); do
      timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null && break
      sleep 0.5
    done
    if timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
      echo "  ✓ 桥就绪"
    else
      echo "  ✗ 桥没起来，看 $LOG/foxglove.log"; sed 's/^/    /' "$LOG/foxglove.log" | tail -6
    fi
  fi
fi

echo "[launcher] 6/7 vins_node"
echo "  ⚠ 现在开始【别碰飞机】—— VINS 初始化需要静止，动了会初始化失败"
# ★ 复跑时总是把旧的停掉重起：估计器不能有两份，而且我们要的是
#   "轨迹从原点重新长出来"——旧 vins_node 的位置早就跑飞了，留着没法看。
if pgrep -f "[v]ins_node" >/dev/null; then
  echo "  已有 vins_node 在跑，先停掉重起（轨迹要从原点重新长）"
  pkill -INT -f "[v]ins_node" 2>/dev/null
  sleep 5
fi
rosrun vins vins_node "$CONFIG" > "$LOG/vins.log" 2>&1 &
sleep 6
if ! pgrep -f "[v]ins_node" >/dev/null; then
  echo "  ✗ vins_node 立刻退出，日志尾部："; sed 's/^/    /' "$LOG/vins.log" | tail -20; exit 1
fi
echo "  ✓ vins_node 在跑"

echo "[launcher] 7/7 诊断转发（Path / 姿态 / |pos|）"
# ★ 必须杀掉重起，不能复用：Path 是【累积在进程内存里的】，
#   旧 relay 里装着上一轮那条飞出去几十公里的轨迹，Foxglove 一打开就是它。
pkill -INT -f "[v]ins_diag_relay" 2>/dev/null && sleep 2
python3 -u "$TOOLS/vins_diag_relay.py" > "$LOG/diag.log" 2>&1 &
sleep 5
if pgrep -f "[v]ins_diag_relay" >/dev/null; then
  echo "  ✓ 转发在跑（发 /diag/{vio_path,imu_path,vio_rpy,grav_rp,att_delta,pos_norm}）"
else
  echo "  ✗ 转发没起来，看 $LOG/diag.log"
fi

echo "[launcher] 复查发射器（相机跑起来之后可能又被重置）"
EM2=$(emitter_state)
if [ "$EM2" = "0" ]; then
  echo "  ✓ emitter_enabled=0 确认"
else
  echo "  ⚠ 读到 ${EM2:-空}，再关一次"
  emitter_off
fi

cat <<EOF

================================================================
  全栈已拉起。逐条验活（★ 不能只看转发层 —— 它在源死之后还会继续发）：
EOF
for t in /vins_estimator/odometry /vins_estimator/imu_propagate \
         /diag/vio_path /diag/imu_path /diag/pos_norm /diag/att_delta; do
  if has_data "$t"; then echo "    ✓ $t"; else echo "    ✗ $t  （静默）"; fi
done

# ---- Foxglove 连接地址：有线优先（与 d430_view 的 start_foxglove.sh 同一套规矩）----
# 有线 192.168.2.110(Pi) <-> 192.168.2.80(笔记本)：实测 11.8MB/s、1.4ms、0% 丢包
# WiFi 192.168.1.215          ：实测  6.4MB/s、   33ms、20% 丢包
# ★ 为什么不用 ping：容器里没装 ping/ip/nc（2026-10-11 实测，只有 curl 和 python3）。
#   判据改读 /proc/net/arp 和 /sys —— 容器是 host 网络模式，读到的就是宿主机的真实状态：
#     ① eth0 载波在（网线插着）
#     ② ARP 表里笔记本有线口是 complete 条目、且走 eth0
#   戳一下 /dev/tcp 是为了 ARP 条目过期时强制重新解析（端口随便，连不上也算数）。
#   ★ 注意别拿 192.168.2.110 当判据 —— 那是板子自己，ping 自己永远成功，测不出网线。
LAPTOP_WIRED=192.168.2.80
PI_WIRED=192.168.2.110
PI_WIFI=192.168.1.215
has_wired() {
    [ "$(cat /sys/class/net/eth0/carrier 2>/dev/null)" = "1" ] || return 1
    timeout 1 bash -c "echo > /dev/tcp/$LAPTOP_WIRED/9" 2>/dev/null
    awk -v ip="$LAPTOP_WIRED" '$1==ip && $3=="0x2" && $6=="eth0"{f=1} END{exit !f}' \
        /proc/net/arp 2>/dev/null
}
if has_wired; then HINT_IP=$PI_WIRED; HINT_LINK=有线; else HINT_IP=$PI_WIFI; HINT_LINK=WiFi; fi

if [ "$ENABLE_FOXGLOVE" = "1" ]; then
  FOX_HINT="  Foxglove（在笔记本上）：
    连接 ws://$HINT_IP:$PORT          ← 本次判定：$HINT_LINK 优先
    布局导入 $TOOLS/foxglove_imu_traj_layout.json"
else
  FOX_HINT="  Foxglove：本次【未启动】（ENABLE_FOXGLOVE=$ENABLE_FOXGLOVE，不传数据）
    要用时在宿主机执行：bash ~/vins_build/foxglove_on.sh"
fi

cat <<EOF

  特征点数已在 3.5 步闸门量过（见上）。运行中想随时复看：
    python3 $TOOLS/feat_watch.py 30

$FOX_HINT

  看什么：
    · 3D 面板：红(VIO)/绿(纯IMU) 两条拖尾。贴合=正常，绿线自己飞走=视觉失效
    · |pos|：静止时都该贴在 0.05 米上下；开始单向上扬就是发散，不用等它飞到几公里
    · /diag/pos_norm 的 x≈y ⇒ 视觉贡献归零（这是最直接的判据）
    · /diag/att_delta 静止应≈0（单位已是度，别再 math.degrees）

  停止：在本脚本的终端按 Ctrl-C
  日志：$LOG/{camera,vins,diag,emitter_guard,foxglove,imu_super}.log
================================================================

EOF

wait
