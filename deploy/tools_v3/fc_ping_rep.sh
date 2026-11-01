#!/bin/bash
# fc_ping.py 到底在测什么 —— 重复采样版。
#
# 为什么重做：我先写了个"桥在跑 vs 桥停掉，各测一次"的对照（fc_ping_ab.sh），
# 结果两次跑出【互相矛盾】的结论：
#     bgfslnutl  桥在跑 → 退出码 1
#     bb5gk1zev  桥在跑 → 退出码 0   ← 同一个 pid 33343，同样条件
# 一次一个样本根本分不出"真差异"和"碰运气"。所以这里每个条件采 6 次。
#
# 假说：px4_imu_node 正以 ~196 Hz 排空 /dev/ttyACM0，fc_ping 再开第二个读者，
#       两个读者把 CDC 字节流劈开，心跳包只要被劈成两半，两边都拼不出来。
#       ⇒ 心跳【间歇性】丢失，不是稳定失败，也不是"串口被独占"（那会每次都
#         报 OSError，而不是超时）。
#
# 判决：桥在跑时失败率显著 >0，桥停掉后 6/6 全过 ⇒ 假说成立，
#       说明【预检必须在桥没跑的时候做】。
#
# ★ 本脚本结束时把桥停在【停】的状态，串口是空的 —— 正好是启动器 step 0
#   预检的干净前置条件。
# ★ 全程 SIGINT，不用 kill -9。
set +u          # ★ 不要 set -u：ROS 的 profile.d/10.roslaunch.sh 引用未定义的 ROS_MASTER_URI
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

TOOLS=/root/xu_drone/live
N=6

run_samples() {
  local label="$1" pass=0 fail=0 i rc
  for i in $(seq 1 $N); do
    out=$(timeout 30 python3 "$TOOLS/fc_ping.py" 2>&1); rc=$?
    if [ "$rc" = "0" ]; then
      pass=$((pass+1)); printf "    [%s] #%d 退出码 0（有心跳）\n" "$label" "$i"
    else
      fail=$((fail+1)); printf "    [%s] #%d 退出码 %s  ← %s\n" "$label" "$i" "$rc" "$out"
    fi
  done
  echo "  ⇒ $label: 过 $pass / 败 $fail （共 $N）"
  echo "$pass" > /tmp/fc_rep_$label.count
}

echo "==================== 条件 1：桥【在】跑 ===================="
pgrep -af "[p]x4_imu_node" | sed 's/^/  桥: /' || echo "  桥: 不在跑（那这个条件做不了）"
run_samples bridge_on

echo
echo "==================== 停掉桥（SIGINT）===================="
pkill -INT -f "[p]x4_imu_node" 2>/dev/null && echo "  → 已发 SIGINT" || echo "  桥本来就没跑"
pkill -INT -f "[w]ait_fc_then_start_imu" 2>/dev/null && echo "  → 监督脚本也发了" || true
for i in $(seq 1 15); do
  pgrep -f "[p]x4_imu_node" >/dev/null 2>&1 || break
  sleep 1
done
if pgrep -af "[p]x4_imu_node" | grep -v pgrep; then
  echo "  ★ 桥还在，串口仍被占；下面这个条件不成立"
else
  echo "  ✓ 桥已退出，/dev/ttyACM0 现在没有读者"
fi

echo
echo "==================== 条件 2：桥【不在】===================="
run_samples bridge_off

echo
echo "==================== 判决 ===================="
p1=$(cat /tmp/fc_rep_bridge_on.count 2>/dev/null || echo 0)
p2=$(cat /tmp/fc_rep_bridge_off.count 2>/dev/null || echo 0)
echo "  桥在跑  : 过 $p1 / $N"
echo "  桥停掉  : 过 $p2 / $N"
if [ "$p1" -lt "$N" ] && [ "$p2" -eq "$N" ]; then
  echo "  ★ 成立：fc_ping 测的不是飞控死活，是【串口有没有别人在抢】。"
  echo "    ⇒ 预检必须在桥没跑的时候做；桥已经跑着的时候预检会随机误报。"
elif [ "$p1" -eq "$N" ] && [ "$p2" -eq "$N" ]; then
  echo "  ⇒ 两边都全过。这一轮的失败要么是极小概率，要么另有原因（比如正好撞上重枚举）。"
  echo "    ★ 那就不能怪抢串口，得回去查 B 轮那两次为什么失败。"
elif [ "$p1" -eq 0 ] && [ "$p2" -eq 0 ]; then
  echo "  ★ 两边【都】失败：飞控是真的没心跳（跟抢串口无关）。要查飞控侧。"
else
  echo "  ⇒ 两边都有失败：更像"心跳本身间歇"或"USB 链路在抖"，不是单纯的抢串口。"
fi
