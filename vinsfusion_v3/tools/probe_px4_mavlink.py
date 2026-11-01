#!/usr/bin/env python3
"""探针：直接听飞控串口，看清 PX4 到底在吐什么。

回答三个问题（这三个不定下来，后面接 IMU 就是瞎接）：
  1. PX4 默认在流哪些消息？各自多少 Hz？
  2. IMU 消息（HIGHRES_IMU / RAW_IMU / SCALED_IMU）在不在流里？多快？
  3. 时间戳干不干净 —— dt 是稳的，还是忽快忽慢/有冻结？

时间戳那一项最关键：VINS 的 IMU 预积分假设 dt 准确。如果 dt 忽大忽小，
再好的标定也救不回来（你们在 OpenVINS 上已经被"100 个样本压成同一时刻"坑过一次）。

用法（容器内）: python3 probe_px4_mavlink.py [/dev/ttyACM0] [被动听秒数=8] [请求后听秒数=6]
"""
import sys
import time
from collections import defaultdict

import numpy as np
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
T_PASSIVE = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
T_ACTIVE = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0

IMU_MSGS = {'HIGHRES_IMU', 'RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3'}


def listen(mav, seconds, label):
    """听一段。

    返回 (stamps, sensor)：
      stamps[name] = 本机【接收】时刻
      sensor[name] = 飞控自己打的【采样】时刻（time_usec，秒）

    这两个必须分开看，因为它们是两件事：
      · 接收时刻的 dt 里有串口+调度抖动 —— 这是"传输"的质量
      · 飞控采样时刻的 dt 才是"传感器"的真实节奏 —— 这是 VINS 预积分需要的

    如果采样 dt 很干净而接收 dt 很脏，就说明必须用传感器时钟映射（香橙派那个桥的做法），
    直接拿接收时刻打戳会把抖动灌进预积分。
    """
    counts = defaultdict(int)
    stamps = defaultdict(list)
    sensor = defaultdict(list)
    t0 = time.time()
    while time.time() - t0 < seconds:
        msg = mav.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        name = msg.get_type()
        if name == 'BAD_DATA':
            continue
        counts[name] += 1
        if name in IMU_MSGS or name in ('ATTITUDE', 'HEARTBEAT'):
            stamps[name].append(time.time())
            us = getattr(msg, 'time_usec', None)
            if us is None:
                us = getattr(msg, 'time_boot_ms', None)
                if us is not None:
                    us = us * 1e3
            if us:
                sensor[name].append(us * 1e-6)
    dur = time.time() - t0
    print(f'--- {label}（{dur:.1f}s）---')
    # 顺带报一下 sysid/compid，确认是飞控本身在发
    print(f'  {"消息":<22} {"条数":>7} {"速率(Hz)":>10}')
    for name, c in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f'  {name:<22} {c:>7} {c / dur:>10.2f}')
    return stamps, sensor


print(f'连接 {PORT} ...')
mav = mavutil.mavlink_connection(PORT, baud=115200, timeout=3, autoreconnect=False)
if not mav.wait_heartbeat(timeout=10):
    print('❌ 10 秒内没收到心跳 —— 飞控没在跑固件，或串口被别的程序占着')
    sys.exit(1)
print(f'✅ 心跳到达：sysid={mav.target_system} compid={mav.target_component}')

# ---- 第一轮：被动听，看 PX4 默认给 USB 口配了什么流 ----
passive = listen(mav, T_PASSIVE, '被动听（不改任何设置）')

# ---- 第二轮：主动把 HIGHRES_IMU 拉到 200Hz，再听 ----
# 这就是香橙派那个桥的做法；只在内存里生效，断电即恢复，不改固件、不落盘。
print()
print('请求 HIGHRES_IMU @ 200Hz（MAV_CMD_SET_MESSAGE_INTERVAL, 5000us）...')
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
    5000,   # 微秒
    0, 0, 0, 0, 0)

active = listen(mav, T_ACTIVE, '请求 200Hz 之后')

# ---- 时间戳体检 ----
print()
print('=' * 70)
print('时间戳体检 —— 接收时刻(传输质量) vs 飞控采样时刻(传感器节奏)')
print('=' * 70)
for label, (stamps, sensor) in (('被动', passive), ('请求后', active)):
    for name in sorted(stamps):
        s = np.array(stamps[name])
        if len(s) < 20:
            continue
        n = len(s)
        b = np.array(sensor.get(name, []))
        n = min(n, len(b))
        print(f'[{label}] {name}   n={n}')

        for tag, arr in (('接收时刻 dt', s), ('采样时刻 dt', b)):
            if len(arr) < 20:
                continue
            dt = np.diff(arr[:n] if len(arr) >= n else arr)
            if len(dt) < 10:
                continue
            tiny = (dt < 5e-4).sum()
            big = (dt > 5e-2).sum()
            flag = '⚠️' if (tiny or big) else '✅'
            print(f'    {tag}  中位 {np.median(dt)*1e3:7.3f} ms   '
                  f'标准差 {dt.std()*1e3:7.3f} ms   最大 {dt.max()*1e3:8.3f} ms  '
                  f'{flag} 压扁{tiny} 空洞{big}')
        if len(b) > 20:
            span = b[n - 1] - b[0]
            print(f'    实算速率 = {(n - 1) / span:.2f} Hz （按飞控自己的采样时钟算，最可信）')
        print()

print('判读：')
print('  · 采样时刻 dt 干净、接收时刻 dt 脏  → 必须做传感器时钟映射，别用到达时间打戳')
print('  · 两个都脏                          → 飞控侧本身就慢/丢包，先解决链路')
print('  · 出现"压扁"（dt≈0）                 → 就是 OpenVINS 上踩过的那种时间戳冻结 bug')
