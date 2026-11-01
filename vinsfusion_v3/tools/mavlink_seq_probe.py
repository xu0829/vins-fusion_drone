#!/usr/bin/env python3
"""独占串口 45 秒，把"飞控到底发了没"这件事钉死。

为什么非做不可：光看收到的间隔，有两种病长得一模一样，但药完全相反：

  传输丢包   —— 飞控按固定周期发，中间的在 USB/内核缓冲里丢了。
                药在【我们这边】：串口缓冲、进程调度。
  飞控没发   —— 飞控本来就把 HIGHRES_IMU 挤到别的时隙去了（链路被
                其它消息占着）。药在【飞控那边】：MAV_x_RATE / MAV_x_MODE。

判据是 MAVLink 自带的 seq。**注意 seq 是【每条链路一个全局计数器】，
不分消息类型** —— 按类型过滤再查连续性会看到"每条都有洞"，那是假象
（两次 HIGHRES_IMU 之间本来就插了几十条别的消息）。所以必须把所有消息
按到达顺序合起来查：

  seq 连续  -> 飞控发一条我们收一条，一条没丢。间隔大 = 飞控真的没发那么快
  seq 有洞  -> 洞的大小就是传输层丢的条数，一个字节都不用在猜

顺手把和带宽有关的飞控参数读出来（**只读，不写飞控**）。

必须先停掉 px4_imu_node，否则两个读者会把串口字节撕开，两边都废。

用法:
    python3 mavlink_seq_probe.py /dev/ttyACM0 [秒数=45]
"""
import collections
import sys
import time

from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
MSG_HIGHRES_IMU = 234

NAMES = {v: k.replace('MAVLINK_MSG_ID_', '')
         for k, v in vars(mavutil.mavlink).items()
         if k.startswith('MAVLINK_MSG_ID_')}

print('连接 %s ...' % PORT)
m = mavutil.mavlink_connection(PORT, baud=115200, timeout=3, autoreconnect=False)
if not m.wait_heartbeat(timeout=10):
    sys.exit('10 秒内没收到心跳 —— 串口还被占着？')
print('心跳: sysid=%s compid=%s' % (m.target_system, m.target_component))

m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        MSG_HIGHRES_IMU, 5000, 0, 0, 0, 0, 0)
print('已请求 HIGHRES_IMU @200Hz，开始听 %.0f 秒...\n' % DUR)

counts = collections.Counter()
hires_t = []
raw = []            # (到达时刻, sysid, compid, seq, msgid)
t0 = time.time()
while time.time() - t0 < DUR:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    mid = msg.get_msgId()
    if mid < 0:
        continue
    counts[mid] += 1
    raw.append((time.time(), msg.get_srcSystem(), msg.get_srcComponent(),
                msg.get_seq(), mid))
    if mid == MSG_HIGHRES_IMU:
        hires_t.append(msg.time_usec * 1e-6)
span = time.time() - t0
total = len(raw)

print('=== 各路消息（按条数，前 15）===')
print('  %-26s %8s %9s' % ('消息', '条数', '速率Hz'))
print('  ' + '-' * 46)
for mid, c in counts.most_common(15):
    print('  %-26s %8d %9.1f' % (NAMES.get(mid, 'ID%d' % mid), c, c / span))
print('  %-26s %8d %9.1f  <- 全部加起来' % ('【合计】', total, total / span))

# ---------------- 决定性判据：全局 seq ----------------
print()
print('=== 判决：把所有消息合起来，seq 有没有洞 ===')
for label, idx in (('sysid', 1), ('compid', 2)):
    vals = set(r[idx] for r in raw)
    print('  %s 取值: %s' % (label, sorted(vals)))
print()
print('  为什么必须合起来看：seq 是每条链路一个计数器，')
print('  HIGHRES_IMU 两条之间本来就插着 ATTITUDE 等几十条别的消息。\n')

stream = sorted(raw, key=lambda r: (r[1], r[2], r[0]))
gaps = []
loss = 0
for (a1, s1, c1, q1, m1), (a2, s2, c2, q2, m2) in zip(stream, stream[1:]):
    if (s1, c1) != (s2, c2):
        continue
    d = (q2 - q1) & 0xFF
    if d != 1:
        gaps.append(d)
        loss += max(0, d - 1)

print('  收到 %d 条，seq 不连续 %d 次' % (total, len(gaps)))
if gaps:
    print('  洞大小分布: %s' % dict(collections.Counter(gaps).most_common(12)))
    print('  => 【传输层真丢包】：丢了约 %d 条 (%.2f%%)'
          % (loss, 100.0 * loss / (total + loss)))
else:
    print('  => 【全局 seq 一条不差完全连续】')
    print('     飞控发多少我们收多少，传输层零丢包。')
    print('     HIGHRES_IMU 的间隔大 = 飞控【本来就没发那么快】，药在飞控侧。')

# ---------------- HIGHRES_IMU 的采样时刻 ----------------
if hires_t:
    import numpy as np
    print()
    print('=== HIGHRES_IMU 的飞控采样间隔 ===')
    us = np.round(np.diff(sorted(hires_t)) * 1e6).astype(int)
    c = collections.Counter(us)
    print('  唯一值 %d 种，前 12：' % len(c))
    for v, k in c.most_common(12):
        print('    %8d us  x%-6d %5.1f%%' % (v, k, 100.0 * k / len(us)))
    print('  实测 %.1f Hz（45 秒平均）' % (len(hires_t) / span))
else:
    print('\n  警告：一条 HIGHRES_IMU 都没收到')

print()
print('=== 带宽相关飞控参数（只读）===')
for p in ('MAV_0_RATE', 'MAV_0_MODE', 'MAV_1_RATE', 'MAV_1_MODE',
          'MAV_2_RATE', 'MAV_2_MODE', 'SER_TEL1_BAUD', 'SER_TEL2_BAUD',
          'IMU_INTEG_RATE'):
    try:
        m.param_fetch_one(p)
        r = None
        t_p = time.time()
        while time.time() - t_p < 1.5:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if r is not None and r.param_id == p:
                break
        if r is not None and r.param_id == p:
            print('  %-16s = %s' % (p, r.param_value))
        else:
            print('  %-16s = (无响应)' % p)
    except Exception as e:  # noqa: BLE001
        print('  %-16s = 读取失败 %s' % (p, e))
m.close()
