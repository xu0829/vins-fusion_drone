#!/usr/bin/env python3
"""探一下飞控在不在：收到心跳退出码 0，没收到退出码 1。

给 wait_fc_then_start_imu.sh 当探针用。刻意保持极简——只做一件事，
免得探针自己出问题反而更难查。
"""
import sys

from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

try:
    m = mavutil.mavlink_connection(PORT, baud=115200, timeout=2,
                                   autoreconnect=False)
    hb = m.wait_heartbeat(timeout=8)
    m.close()
except Exception:  # noqa: BLE001  串口打不开/被占，一律当作"飞控不在"
    sys.exit(1)

sys.exit(0 if hb else 1)
