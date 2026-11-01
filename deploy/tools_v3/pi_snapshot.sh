#!/bin/bash
# 容器内一行命令看全所有判据 —— 免得每次诊断来回问七八个 rostopic。
#
# 为什么要有这个：今晚的教训是"仪表必须贴住被测量的信号本身"，
# 而每条判据分散在不同话题/日志里，逐个查既慢又容易漏。
# 这里把五条发散路径的判据一次性打出来，对照着看就差不到哪去。
#
# 用法（容器内）：bash /root/xu_drone/live/pi_snapshot.sh
set -u
source /opt/ros/noetic/setup.bash

LOG=${LOG:-/tmp/vins_pi}
GOOD=80

hr() { printf '%s\n' "------------------------------------------------------------"; }

hr; echo "【1】进程还活着吗"
for p in vins_node px4_imu_node vins_diag_relay emitter_guard roscore foxglove_bridge; do
  # 方括号技巧：模式串写成 [v]ins_node，免得 pgrep 匹配到执行本脚本的这行命令自己
  n=$(pgrep -fc "[${p:0:1}]${p:1}" 2>/dev/null)
  n=${n:-0}
  printf '   %-18s %s\n' "$p" "$([ "$n" -gt 0 ] && echo "✓ 在跑($n)" || echo "✗ 没了")"
done

hr; echo "【2】话题真的在发吗（不能只看转发层——它在源死之后还会继续发）"
for t in /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /imu \
         /vins_estimator/odometry /diag/pos_norm /diag/att_delta; do
  if timeout 4 rostopic echo -n 1 "$t" >/dev/null 2>&1; then
    printf '   ✓ %s\n' "$t"
  else
    printf '   ✗ %-40s 静默\n' "$t"
  fi
done
echo "   —— 图像话题的发布者数（0 = 冻死那条路）："
for t in /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw; do
  printf '      %s -> %s\n' "$t" "$(timeout 5 rostopic info "$t" 2>/dev/null | grep -A1 '^Publishers' | tail -1)"
done

hr; echo "【3】发射器状态（必须一直是 0）"
echo -n "   当前 emitter_enabled = "
timeout 5 rostopic echo -n 1 /camera/stereo_module/parameter_updates 2>/dev/null \
  | grep -A1 '"emitter_enabled"' | tail -1 | tr -d ' ' | sed 's/^value://'
echo "   守卫日志尾部："
sed 's/^/      /' "$LOG/emitter_guard.log" 2>/dev/null | tail -5

hr; echo "【4】VINS 日志里的告警计数（三条路的指纹）"
for k in "numerical unstable" "feature tracking not enough" "waiting for image and imu" "nan"; do
  printf '   %-32s %s 次\n' "$k" "$(grep -ci "$k" "$LOG/vins.log" 2>/dev/null || echo 0)"
done
echo "   vins.log 最后修改: $(stat -c %y "$LOG/vins.log" 2>/dev/null | cut -d. -f1)  现在: $(date '+%F %T')"
echo "   日志尾部："; sed 's/^/      /' "$LOG/vins.log" 2>/dev/null | tail -4

hr; echo "【5】位置与姿态（发散判据）"
python3 - <<'PY' 2>/dev/null
import re
for name in ("vins",):
    try:
        txt = open("/tmp/vins_pi/vins.log", errors="replace").read()
    except Exception:
        print("   读不到 vins.log"); break
    # VINS 打印形如 "p: x y z" / "v: ..." 或 odometry 行；取最后一条含 p: 的
    lines = [l for l in txt.splitlines() if re.search(r"\bp:\s*-?\d", l)]
    if not lines:
        print("   还没有 p: 行（可能未初始化完成）"); break
    last = lines[-1]
    m = re.search(r"p:\s*(-?[\d.eE+]+)\s+(-?[\d.eE+]+)\s+(-?[\d.eE+]+)", last)
    if m:
        x, y, z = (float(m.group(i)) for i in (1, 2, 3))
        print(f"   |pos| = {(x*x+y*y+z*z)**0.5:.4f} m   (x={x:.4f} y={y:.4f} z={z:.4f})")
        print(f"   时间戳: {last.split()[0] if last.split() else '?'}")
    print(f"   共 {len(lines)} 条 p: 行")
PY
echo "   /diag/pos_norm 最后一条（x=|VIO| y=|纯IMU| z=|差|；★x≈y ⇒ 视觉贡献≈0）："
timeout 6 rostopic echo -n 1 /diag/pos_norm 2>/dev/null | grep -E "^\s+(x|y|z):" | tr '\n' ' ' | sed 's/^/      /'; echo
echo "   /diag/att_delta 最后一条（★单位已是度，静止应≈0；15~31 = 发射器污染）："
timeout 6 rostopic echo -n 1 /diag/att_delta 2>/dev/null | grep -E "^\s+(x|y|z|data):" | head -3 | sed 's/^/      /'

hr; echo "【6】特征点数（★低于 $GOOD 就是"视觉饿死"，先开灯换视角再谈别的）"
timeout 12 python3 /root/xu_drone/live/feat_watch.py 6 2>/dev/null | tail -3

hr; echo "【7】IMU 健康"
wc -l /tmp/imu_raw_recover.csv 2>/dev/null | sed 's/^/   CSV 行数: /'
sed 's/^/   /' /tmp/imu_super.log 2>/dev/null | tail -3

hr; echo "【8】USB 有没有掉过（终审：内核说话）"
dmesg 2>/dev/null | grep -icE "usb 2-1.*(disconnect|reset SuperSpeed)|xhci.*(Set TR Deq|failed)" | sed 's/^/   事件数: /'
dmesg 2>/dev/null | grep -iE "usb 2-1|usb 7-1" | tail -4 | sed 's/^/   /'
hr
