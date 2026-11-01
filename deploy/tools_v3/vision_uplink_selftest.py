#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vision_uplink 自检：不碰串口、不碰飞控，只验证"代码在这个容器里能不能跑、
发出去的字节对不对"。

在重启整栈之前跑一次，比重启完才发现 API/协议不对划算得多。

    docker exec vins_pi bash -lc 'source /opt/ros/noetic/setup.bash && \
        python3 /root/xu_drone/live/vision_uplink_selftest.py'
"""
import math
import os
import struct
import sys
import traceback

# 必须最先设：px4_imu_node 里也是这个顺序，自检要复现真实运行条件
os.environ.setdefault('MAVLINK20', '1')

FAIL = []
WARN = []


def check(name, cond, extra=''):
    print('  [%s] %s %s' % ('OK ' if cond else 'FAIL', name, extra))
    if not cond:
        FAIL.append(name)


def warn(name, extra=''):
    print('  [警告] %s %s' % (name, extra))
    WARN.append(name)


print('=== 1. pymavlink 协议与 API ===')
from pymavlink import mavutil  # noqa: E402

proto = mavutil.mavlink.WIRE_PROTOCOL_VERSION
if proto == '2.0':
    check('WIRE_PROTOCOL_VERSION = 2.0', True)
else:
    warn('WIRE_PROTOCOL_VERSION = %s（不是 2.0）—— 102 号会退化成无 covariance 的 v1 报文'
         % proto)

check('MAVLink_vision_position_estimate_message 存在',
      hasattr(mavutil.mavlink, 'MAVLink_vision_position_estimate_message'))
check('MAV_CMD_SET_MESSAGE_INTERVAL = 511',
      getattr(mavutil.mavlink, 'MAV_CMD_SET_MESSAGE_INTERVAL', None) == 511,
      'got %s' % getattr(mavutil.mavlink, 'MAV_CMD_SET_MESSAGE_INTERVAL', None))
check('MAV_CMD_REQUEST_MESSAGE = 512',
      getattr(mavutil.mavlink, 'MAV_CMD_REQUEST_MESSAGE', None) == 512,
      'got %s' % getattr(mavutil.mavlink, 'MAV_CMD_REQUEST_MESSAGE', None))
for nm, want in (('MAVLINK_MSG_ID_ESTIMATOR_STATUS', 230),
                 ('MAVLINK_MSG_ID_LOCAL_POSITION_NED', 32),
                 ('MAVLINK_MSG_ID_ATTITUDE', 30),
                 ('MAVLINK_MSG_ID_AUTOPILOT_VERSION', 148)):
    check('%s = %d' % (nm, want), getattr(mavutil.mavlink, nm, None) == want,
          'got %s' % getattr(mavutil.mavlink, nm, None))

_VPE = mavutil.mavlink.MAVLink_vision_position_estimate_message
check('102 号 crc_extra = 158（PX4 期望值）', _VPE.crc_extra == 158, 'got %s' % _VPE.crc_extra)
check('102 号含 covariance 字段', 'covariance' in _VPE.ordered_fieldnames,
      '%s' % (_VPE.ordered_fieldnames,))

print('\n=== 2. 字节级：真造一条 102 并逐字段核对 ===')
# 按 PX4 的读法核对：载荷偏移 0=usec(8B) 8..32=6个float(rpy+xyz) 32..116=covariance[21]
try:
    usec, x, y, z, roll, pitch, yaw = 1791389123456789, 0.5, -1.25, -0.75, 0.01, -0.02, 1.5708
    cov = [0.0] * 21
    cov[0] = cov[6] = cov[11] = 0.04      # 位置 x/y/z 方差
    cov[15] = cov[18] = cov[20] = 0.03    # 姿态 roll/pitch/yaw 方差
    msg = _VPE(usec, x, y, z, roll, pitch, yaw, cov, 0)
    pkt = msg.pack(mavutil.mavlink.MAVLink(None))

    check('帧头 0xFD（MAVLink 2）', pkt[0] == 0xFD, 'got 0x%02X' % pkt[0])
    check('载荷长 = 116 字节', pkt[1] == 116, 'got %d' % pkt[1])
    check('msgid = 102', pkt[7] | (pkt[8] << 8) | (pkt[9] << 16) == 102,
          'got %d' % (pkt[7] | (pkt[8] << 8) | (pkt[9] << 16)))

    payload = pkt[10:10 + 116]
    u_, px, py, pz, r_, p_, y_ = struct.unpack('<Q6f', payload[:32])
    check('usec 解出来一致', u_ == usec, 'got %d' % u_)
    check('x/y/z 解出来一致',
          abs(px - x) < 1e-6 and abs(py - y) < 1e-6 and abs(pz - z) < 1e-6,
          'got (%.4f, %.4f, %.4f)' % (px, py, pz))
    check('roll/pitch/yaw 解出来一致',
          abs(r_ - roll) < 1e-6 and abs(p_ - pitch) < 1e-6 and abs(y_ - yaw) < 1e-6,
          'got (%.5f, %.5f, %.5f)' % (r_, p_, y_))
    c = struct.unpack('<21f', payload[32:116])
    check('covariance 落在偏移 32（PX4 就在这里读）',
          abs(c[0] - 0.04) < 1e-9 and abs(c[6] - 0.04) < 1e-9 and abs(c[11] - 0.04) < 1e-9
          and abs(c[15] - 0.03) < 1e-9 and abs(c[18] - 0.03) < 1e-9 and abs(c[20] - 0.03) < 1e-9,
          '[0]=%.3f [6]=%.3f [11]=%.3f [15]=%.3f [18]=%.3f [20]=%.3f'
          % (c[0], c[6], c[11], c[15], c[18], c[20]))
    check('非对角线项为 0', all(abs(v) < 1e-12 for i, v in enumerate(c) if i not in (0, 6, 11, 15, 18, 20)))
except Exception:
    check('构造/打包/解析 102 号消息', False)
    traceback.print_exc()

print('\n=== 3. 坐标变换数值（8 个用例）===')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vision_uplink as V  # noqa: E402


def rot(axis, deg):
    h = math.radians(deg) / 2.0
    s, c = math.sin(h), math.cos(h)
    return {'x': (s, 0, 0, c), 'y': (0, s, 0, c), 'z': (0, 0, s, c)}[axis]


def ned_of(q_wb, p_enu):
    ned = (p_enu[1], p_enu[0], -p_enu[2])
    q = V.qmul(V.qmul(V.Q_ENU2NED, q_wb), V.Q_FLU2FRD)
    return ned, tuple(math.degrees(v) for v in V.q_to_rpy(q))


cases = [
    ('位置 (E,N,U)=(1,2,3) → NED=(2,1,-3)', ned_of((0, 0, 0, 1), (1, 2, 3))[0], (2, 1, -3)),
    ('机头朝东 → yaw=+90', ned_of((0, 0, 0, 1), (0, 0, 0))[1], (0.0, 0.0, 90.0)),
    ('机头朝北 → yaw=0', ned_of(rot('z', 90), (0, 0, 0))[1], (0.0, 0.0, 0.0)),
    ('机头朝西 → yaw=-90', ned_of(rot('z', 180), (0, 0, 0))[1], (0.0, 0.0, -90.0)),
    ('低头10° → pitch=-10（PX4 鼻上为正）', ned_of(rot('y', 10), (0, 0, 0))[1], (0.0, -10.0, 90.0)),
    ('右滚10° → roll=+10（PX4 右滚为正）', ned_of(rot('x', 10), (0, 0, 0))[1], (10.0, 0.0, 90.0)),
    ('朝北+低头10° → (0,-10,0)', ned_of(V.qmul(rot('z', 90), rot('y', 10)), (0, 0, 0))[1],
     (0.0, -10.0, 0.0)),
    ('rpy→四元数→rpy 往返',
     tuple(round(math.degrees(v), 6) for v in V.q_to_rpy(V.rpy_to_q(0.3, -0.2, 1.1))),
     (round(math.degrees(0.3), 6), round(math.degrees(-0.2), 6), round(math.degrees(1.1), 6))),
]
for name, got, want in cases:
    ok = all(abs(a - b) < 1e-5 for a, b in zip(got, want))
    check(name, ok, 'got %s' % (tuple(round(v, 4) for v in got),))
for q in (rot('x', 10), rot('y', 33), rot('z', -77)):
    n = sum(v * v for v in q) ** 0.5
    check('四元数单位长度', abs(n - 1) < 1e-12, 'norm=%.12f' % n)

print('\n=== 4. 模式文件与接口 ===')
u = V.VisionUplink.__new__(V.VisionUplink)     # 不跑 __init__（那要 roscore）
u.mode = V.VisionUplink._read_mode_file(u)
check('读到合法模式 0/1/2/3', u.mode in (0, 1, 2, 3), 'mode=%d（文件 %s）' % (u.mode, V.MODE_FILE))
check('mode=%d 时 extra_types 长度符合预期' % u.mode,
      len(V.VisionUplink.extra_types(u)) == (len(V.VisionUplink.EXTRA_TYPES) if u.mode else 0),
      'len=%d' % len(V.VisionUplink.extra_types(u)))
check('tick / on_mavlink / on_reconnect / extra_types / _make_vpe / _make_odometry 都在',
      all(hasattr(V.VisionUplink, n)
          for n in ('tick', 'on_mavlink', 'on_reconnect', 'extra_types',
                    '_make_vpe', '_make_odometry')))
check('MODE_DESC 覆盖 0..3', sorted(V.MODE_DESC) == [0, 1, 2, 3], '%s' % (V.MODE_DESC,))
check('331 的帧常量：LOCAL_FRD=20 / ESTIMATOR_TYPE_VIO=3',
      V.MAV_FRAME_LOCAL_FRD == 20 and V.MAV_ESTIMATOR_TYPE_VIO == 3,
      'FRD=%s VIO=%s' % (V.MAV_FRAME_LOCAL_FRD, V.MAV_ESTIMATOR_TYPE_VIO))

print('\n=== 5. ROS 消息类型可导入 ===')
try:
    from geometry_msgs.msg import PoseStamped   # noqa: F401
    from nav_msgs.msg import Odometry            # noqa: F401
    from std_msgs.msg import String              # noqa: F401
    check('geometry_msgs / nav_msgs / std_msgs', True)
except Exception as e:  # noqa: BLE001
    check('geometry_msgs / nav_msgs / std_msgs', False, str(e))

print('\n=== 6. _make_vpe 走真路径（covariance 必须真的在报文里）===')
try:
    u.vpe_fields = len(_VPE.ordered_fieldnames)
    # 用 tick() 里真实的协方差常量，而不是全 0
    cov_real = [0.0] * 21
    cov_real[0] = cov_real[6] = cov_real[11] = V.POS_VAR
    cov_real[15] = cov_real[18] = cov_real[20] = V.ANG_VAR
    check('POS_VAR / ANG_VAR 都是正数（0 = 无限自信，代码会拒）',
          V.POS_VAR > 0 and V.ANG_VAR > 0, 'POS=%.4f ANG=%.4f' % (V.POS_VAR, V.ANG_VAR))

    m2 = V.VisionUplink._make_vpe(u, 123456789, (1.0, 2.0, -3.0), (0.0, 0.0, 0.0), cov_real)
    p2 = m2.pack(mavutil.mavlink.MAVLink(None))
    check('_make_vpe 载荷 = 116 字节', p2[1] == 116, 'got %d' % p2[1])
    c2 = struct.unpack('<21f', p2[10 + 32:10 + 116])
    check('_make_vpe 里协方差非零且位置正确',
          abs(c2[0] - V.POS_VAR) < 1e-9 and abs(c2[20] - V.ANG_VAR) < 1e-9,
          '[0]=%.4f [20]=%.4f' % (c2[0], c2[20]))

    # 记录一条 MAVLink 2 的行为（不是 bug）：尾部"等于默认值"的字段会被裁掉，
    # 接收端补零。所以协方差必须非零，否则等于没发。
    m3 = V.VisionUplink._make_vpe(u, 123456789, (1.0, 2.0, -3.0), (0.0, 0.0, 0.0), [0.0] * 21)
    p3 = m3.pack(mavutil.mavlink.MAVLink(None))
    check('（已知行为）全零协方差会被 v2 裁成 %d 字节，接收端补零' % p3[1],
          p3[1] < 116, '这就是为什么 _env_var() 拒绝 <=0 的方差')
except Exception:
    check('_make_vpe', False)
    traceback.print_exc()

print('\n=== 7. δ 对齐 + _process_odom 真路径（不发串口，master 是假的）===')
import collections  # noqa: E402

LOGS = []
V.rospy.loginfo = lambda *a, **k: LOGS.append((a[0] % a[1:]) if len(a) > 1 else str(a[0]))
V.rospy.logwarn = lambda *a, **k: LOGS.append('[WARN] ' + ((a[0] % a[1:]) if len(a) > 1 else str(a[0])))


class NS(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Msg(NS):
    def __init__(self, typ, **kw):
        NS.__init__(self, **kw)
        self._typ = typ

    def get_type(self):
        return self._typ


class FakePub(object):
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class FakeMaster(object):
    def __init__(self):
        self.sent = []
        self.mav = self
        self.target_system = 1
        self.target_component = 1

    def send(self, m):
        self.sent.append(m)


def mk_odom(t, q_wb, p_enu):
    """伪造 nav_msgs/Odometry 的最小可用替身。"""
    return NS(header=NS(stamp=NS(to_sec=lambda: t)),
              pose=NS(pose=NS(position=NS(x=p_enu[0], y=p_enu[1], z=p_enu[2]),
                              orientation=NS(x=q_wb[0], y=q_wb[1], z=q_wb[2], w=q_wb[3]))))


def mk_uplink(mode=2):
    """不走 __init__（那要 roscore），把 __init__ 会设的字段照抄一份。

    故意手抄：将来往 __init__ 里加字段而忘了改这里，测试会以 AttributeError 炸出来。
    """
    u = V.VisionUplink.__new__(V.VisionUplink)
    u.master = FakeMaster()
    u.mode = mode
    u._t_mode = u._t_status = 0.0
    u._started = True
    u.odom = None
    u.sent_seq = 0
    for n in ('n_sent', 'n_skip_bad', 'n_skip_late', 'n_no_ref', 'n_stale_ref'):
        setattr(u, n, 0)
    u.t_first = 0.0
    u.last_latency = float('nan')
    u.ekf_ned = None
    u.att_rpy = None
    u.att_t = 0.0
    u.att_tb_ms = 0
    u.est_flags = 0
    u.fc_version = None
    u.armed = None
    u.sys_status = None
    u._last_sys_line = None
    u._last_stext = ''
    u._last_stext_n = 0
    u.n_statustext = 0
    u.dyaw = None
    u._dyaw_buf = collections.deque(maxlen=240)
    u._psi_fc_prev = None
    u._pos_prev = None
    u.n_no_dyaw = 0
    u.odom_reset_n = 0
    u._t_fresh = 0.0
    u._t_late_log = 0.0
    u._pose = None
    u._pose3 = None
    u._n_sent_win = 0
    u._t_win = 0.0
    u.csv_vision = None
    u.csv_telem = None
    u.vpe_fields = len(_VPE.ordered_fieldnames)
    for n in ('pub_stext', 'pub_status', 'pub_ned'):
        setattr(u, n, FakePub())
    return u


check('wrap_pi 端点', abs(V.wrap_pi(0.0)) < 1e-12
      and abs(V.wrap_pi(3 * math.pi / 2) + math.pi / 2) < 1e-12
      and abs(V.wrap_pi(-3 * math.pi / 2) - math.pi / 2) < 1e-12)
check('SYS_SENSOR_BITS 含磁力计位',
      any('磁' in lb for _, lb in V.SYS_SENSOR_BITS),
      '%s' % (V.SYS_SENSOR_BITS,))

FC_REF, HOST_REF = 1.0e9, 1000.0           # 飞控 1000s 开机时刻 ↔ 宿主 epoch 1000.0
P_NED_V = (3.0, 4.0, -1.0)                 # VINS 世界系里的 NED 位置
P_ENU = (P_NED_V[1], P_NED_V[0], -P_NED_V[2])   # 同一位置的 ENU 表达


def run_odom(u, i, psi_v_deg, psi_fc_deg, p_enu=P_ENU, dt=0.0667):
    """喂一帧 odom（+ 配对的 ATTITUDE）再跑一次 _process_odom，返回这一帧的飞控时刻。

    ref 每帧都刷新：真实系统里它是"最近一条 HIGHRES_IMU 的 (fc, host) 对"，跟着 IMU
    桥一直在走。钉死不动会被 REF_STALE_S 判成过旧而停发 —— 那是它该做的事（本自检
    第一版就栽在这儿：δ 只收到 28 个样本就断流，正好卡在 MIN_N=30 之前）。

    odom 的四元数是 ENU/FLU 的：**恒等 = 机头朝东 = NED yaw 90°**（不是 0°）。
    所以要做出 NED 的 ψ_v，得绕 FLU 的 z 转 (90° − ψ_v)。
    """
    t = HOST_REF + 0.05 + i * dt
    now = t + 0.1
    fc_ref = FC_REF + (t - HOST_REF) * 1e6
    ref = (fc_ref, t, 0.0)
    if psi_fc_deg is not None:
        u.att_rpy = (0.0, 0.0, math.radians(psi_fc_deg))
        u.att_tb_ms = int(round(fc_ref / 1000.0))
        u.att_t = now
    h = math.radians(90.0 - psi_v_deg) * 0.5
    q_enu = (0.0, 0.0, math.sin(h), math.cos(h))
    u.odom = (mk_odom(t, q_enu, p_enu), now, i + 1)
    u._process_odom(ref, now)
    return fc_ref


# --- 7a 静止：ψ_v=90°（VINS 初始化朝向），ψ_fc=40° ⇒ δ=−50°
u = mk_uplink(2)
for i in range(60):
    run_odom(u, i, 90.0, 40.0)
check('δ 收敛到 −50°', u.dyaw is not None and abs(math.degrees(u.dyaw) + 50.0) < 0.01,
      'got %s' % ('None' if u.dyaw is None else '%.4f°' % math.degrees(u.dyaw)))
check('δ 收敛时打了说明日志',
      any('δ 对齐完成' in s for s in LOGS),
      next((s for s in LOGS if 'δ 对齐完成' in s), '（没有）'))
check('mode=2 且 δ 有效 ⇒ 真的发了', len(u.master.sent) > 0, '发了 %d 帧' % len(u.master.sent))

m = u.master.sent[-1]
dd = math.radians(40.0 - 90.0)
want = (math.cos(dd) * P_NED_V[0] - math.sin(dd) * P_NED_V[1],
        math.sin(dd) * P_NED_V[0] + math.cos(dd) * P_NED_V[1],
        P_NED_V[2])
check('发出去的位置 = Rz(δ)·p_ned',
      abs(m.x - want[0]) < 1e-6 and abs(m.y - want[1]) < 1e-6 and abs(m.z - want[2]) < 1e-6,
      'got (%.4f, %.4f, %.4f) 期望 (%.4f, %.4f, %.4f)'
      % (m.x, m.y, m.z, want[0], want[1], want[2]))
check('发出去的 yaw = ψ_fc（转完 δ 后应等于飞控）',
      abs(math.degrees(m.yaw) - 40.0) < 1e-3, 'got %.5f°' % math.degrees(m.yaw))
check('发出去的 roll/pitch 仍为 0',
      abs(m.roll) < 1e-9 and abs(m.pitch) < 1e-9,
      'got (%.6f, %.6f)' % (m.roll, m.pitch))
want_usec = FC_REF + (0.05 + 59 * 0.0667) * 1e6      # 第 59 帧的飞控时刻
check('usec 用的是飞控时间基（不是宿主 epoch）',
      abs(m.usec - want_usec) < 2000,
      'got %d 期望约 %d' % (m.usec, want_usec))

# --- 7b 缓慢转动：两路 yaw 一起走，δ 靠"同一物理时刻配对"保持不变
u2 = mk_uplink(2)
for i in range(60):
    run_odom(u2, i, 90.0 + 0.5 * i, 40.0 + 0.5 * i)
check('转动中 δ 仍 = −50°（配对按物理时刻，不是按到达时刻）',
      u2.dyaw is not None and abs(math.degrees(u2.dyaw) + 50.0) < 0.01,
      'got %s' % ('None' if u2.dyaw is None else '%.4f°' % math.degrees(u2.dyaw)))

# --- 7c δ=0：yaw 本来就对齐时不能乱转
u3 = mk_uplink(2)
for i in range(60):
    run_odom(u3, i, 90.0, 90.0)
m3 = u3.master.sent[-1] if u3.master.sent else None
check('δ=0 时位置原样发出（不多转一下）',
      m3 is not None and abs(m3.x - P_NED_V[0]) < 1e-9 and abs(m3.y - P_NED_V[1]) < 1e-9,
      'got %s' % (None if m3 is None else '(%.4f, %.4f)' % (m3.x, m3.y)))

# --- 7d 没有可配对的 ATTITUDE ⇒ δ 无效 ⇒ mode=2 一帧都不发
u4 = mk_uplink(2)
for i in range(60):
    run_odom(u4, i, 90.0, None)          # 不给 ATTITUDE
check('δ 测不出来时不发（宁可不发也不喂转过的位置）',
      u4.dyaw is None and len(u4.master.sent) == 0 and u4.n_no_dyaw == 60,
      'dyaw=%s 发=%d n_no_dyaw=%d' % (u4.dyaw, len(u4.master.sent), u4.n_no_dyaw))

# --- 7e odom 位置突跳 ⇒ δ 作废（VINS 重开/回环）
# 突跳时先清空缓冲，紧接着本帧又采了一个样本（当场开始重测）⇒ 事后 buf=1 是正常的。
LOGS[:] = []
n_before = len(u._dyaw_buf)
run_odom(u, 60, 90.0, 40.0, p_enu=(P_ENU[0] + 5.0, P_ENU[1], P_ENU[2]))
check('odom 位置跳 5m ⇒ δ 作废重测',
      u.dyaw is None and len(u._dyaw_buf) <= 1 and n_before >= 30
      and any('δ 作废重测' in s for s in LOGS),
      'dyaw=%s buf=%d(原 %d) %s'
      % (u.dyaw, len(u._dyaw_buf), n_before, [s for s in LOGS if '跳变' in s][:1]))

# --- 7g 状态行必须报【当前】延迟，不能报最后发出去那帧的
# （2026-10-07 实测踩到：VINS 落后 2~4 秒、一帧都发不出去，屏幕却一直印"延迟=492ms"）
LOGS[:] = []
u7 = mk_uplink(2)
for i in range(60):
    run_odom(u7, i, 90.0, 40.0)
n_before = len(u7.master.sent)
tL = HOST_REF + 0.05 + 60 * 0.0667
late_now = tL + 3.0                       # 这一帧迟到了 3 秒
u7.odom = (mk_odom(tL, (0.0, 0.0, 0.0, 1.0), P_ENU), late_now, 999)
u7._process_odom((FC_REF + (tL - HOST_REF) * 1e6, tL, 0.0), late_now)
u7._t_status = 0.0
u7._status_line(late_now)
line = [s for s in LOGS if 'mode=2' in s][-1] if any('mode=2' in s for s in LOGS) else ''
check('迟到的帧不发（延迟闸生效）', len(u7.master.sent) == n_before,
      '发=%d（之前 %d）' % (len(u7.master.sent), n_before))
check('状态行报当前延迟 3000ms（不是上次发出那帧的 100ms）+ 标出已停发',
      '延迟=3000ms' in line and '已停发' in line, line)

# --- 7f 飞控控制台/健康镜像（on_mavlink 的完整分发链）
LOGS[:] = []
u.on_mavlink(Msg('STATUSTEXT', severity=6, text=b'ekf2: starting EV height fusion\x00\x00'))
u.on_mavlink(Msg('STATUSTEXT', severity=4, text=b'ekf2: stopping EV position fusion, continuing conditions failing\x00\x00'))
check('STATUSTEXT 被镜像且关键词被高亮',
      any('★' in s and 'EV height' in s for s in LOGS)
      and any('EV position' in s for s in LOGS),
      '%s' % ([s for s in LOGS if '[FC]' in s][:2],))
LOGS[:] = []
for _ in range(25):
    u.on_mavlink(Msg('STATUSTEXT', severity=6, text=b'same text again\x00'))
check('同一条刷屏被折叠（25 条 → 2 条）',
      len([s for s in LOGS if '[FC]' in s]) == 2,
      '打了 %d 条' % len([s for s in LOGS if '[FC]' in s]))

LOGS[:] = []
u.on_mavlink(Msg('SYS_STATUS', onboard_control_sensors_present=0,
                 onboard_control_sensors_enabled=0, onboard_control_sensors_health=0))
check('SYS_STATUS 能报出"没有磁力计"（这正是 yaw_align 的前提）',
      any('磁力计=无' in s for s in LOGS), '%s' % (LOGS[:1],))
LOGS[:] = []
mag_bit = 4   # MAV_SYS_STATUS_SENSOR_3D_MAG
u.on_mavlink(Msg('SYS_STATUS', onboard_control_sensors_present=mag_bit,
                 onboard_control_sensors_enabled=mag_bit, onboard_control_sensors_health=mag_bit))
check('SYS_STATUS 也能报出磁力计健康',
      any('磁力计=健康' in s for s in LOGS), '%s' % (LOGS[:1],))
LOGS[:] = []
u.on_mavlink(Msg('HEARTBEAT', base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
u.on_mavlink(Msg('HEARTBEAT', base_mode=0))
check('HEARTBEAT 解锁状态被镜像（已解锁会告警）',
      u.armed is False and any('★已解锁★' in s for s in LOGS) and any('已上锁' in s for s in LOGS),
      '%s' % (LOGS,))

print('\n=== 8. 331 号 ODOMETRY（mode=3）：字节布局 + PX4 侧能不能还原出真值 ===')
_ODOM = mavutil.mavlink.MAVLink_odometry_message
check('331 号 crc_extra = 91（PX4 期望值）', _ODOM.crc_extra == 91, 'got %s' % _ODOM.crc_extra)
check('331 号有 frame_id / child_frame_id / quality',
      all(n in _ODOM.ordered_fieldnames
          for n in ('frame_id', 'child_frame_id', 'quality', 'reset_counter')))

# --- 独立实现（故意不复用 vision_uplink 的数学，避免"自己验自己"）---
def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qinv(q):
    return (-q[0], -q[1], -q[2], q[3])


def _dcm(q):
    """R = Dcmf(q)：PX4 的约定，v_world = R·v_body。"""
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _mv(R, v):
    return tuple(sum(R[i][j] * v[j] for j in range(3)) for i in range(3))


def _q_z(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _norm(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def _same_rot(a, b, tol=1e-6):
    """q 与 −q 表示同一个旋转：允许【整体】符号翻转，但不允许单分量翻。"""
    return min(max(abs(x - y) for x, y in zip(a, b)),
               max(abs(x + y) for x, y in zip(a, b))) < tol


# 真值：机体真实姿态（body→真北 NED）和真实位置
Q_EKF_TRUE = _norm(_qmul(_q_z(37.0), _qmul((math.sin(math.radians(6) / 2), 0, 0, math.cos(math.radians(6) / 2)),
                                          (0, math.sin(math.radians(-11) / 2), 0, math.cos(math.radians(-11) / 2)))))
P_EKF_TRUE = (1.75, -2.4, -0.6)          # 真北 NED，米

# VINS 世界系 = 真北 NED 绕 z 转 psi_v（VINS 初始化那一刻的机头朝向）。
# 送出去的量全在 VINS 系里：p_ev = Rz(-psi_v)·p_true，q_ev = Qz(-psi_v)⊗q_true。
for psi_v in (0.0, -62.6, 90.0, 173.0):
    p_ev = _mv(_dcm(_q_z(-psi_v)), P_EKF_TRUE)
    q_ev = _qmul(_q_z(-psi_v), Q_EKF_TRUE)
    u5 = mk_uplink(3)
    u5.odom_reset_n = 7
    m = V.VisionUplink._make_odometry(u5, 1791389123456789, p_ev, q_ev,
                                      [V.POS_VAR] * 21)
    pkt = m.pack(mavutil.mavlink.MAVLink(None))
    tag = 'ψ_v=%+.1f°' % psi_v
    pl = pkt[10:10 + 233]
    t_u, = struct.unpack_from('<Q', pl, 0)
    xyz = struct.unpack_from('<3f', pl, 8)
    # 线序：MAVLink 的 q[4] 规定是 [w,x,y,z]，而本文件其余地方一律 (x,y,z,w)。
    # 这一行换序不能省 —— 少了它，自检和被检代码就"同错自洽"：2026-10-09 的
    # 四元数序 bug 正是这么一路通过自检的（下面 PX4 侧复算直接用 qq）。
    q_wire = struct.unpack_from('<4f', pl, 20)
    qq = (q_wire[1], q_wire[2], q_wire[3], q_wire[0])
    vvv = struct.unpack_from('<3f', pl, 36)
    rpy_rate = struct.unpack_from('<3f', pl, 48)
    pcov = struct.unpack_from('<21f', pl, 60)
    vcov = struct.unpack_from('<21f', pl, 144)
    fid, cid, rst, est, qua = struct.unpack_from('<5B', pl, 228)

    if abs(psi_v + 62.6) < 1e-9:          # 只在一个用例上做完整字节核对
        check('331 帧头 0xFD、msgid=331、载荷 %d 字节（无尾部裁剪）' % pkt[1],
              pkt[0] == 0xFD and pkt[1] == 233
              and pkt[7] | (pkt[8] << 8) | (pkt[9] << 16) == 331,
              'len=%d id=%d' % (pkt[1], pkt[7] | (pkt[8] << 8) | (pkt[9] << 16)))
        # pymavlink 自己认为的线序必须与上面硬编码的偏移一致（否则是 dialect 变了）
        want_order = ['time_usec', 'x', 'y', 'z', 'q', 'vx', 'vy', 'vz',
                      'rollspeed', 'pitchspeed', 'yawspeed',
                      'pose_covariance', 'velocity_covariance',
                      'frame_id', 'child_frame_id', 'reset_counter',
                      'estimator_type', 'quality']
        check('331 线序与硬编码偏移一致（pymavlink 的 ordered_fieldnames）',
              list(_ODOM.ordered_fieldnames) == want_order,
              '%s' % (list(_ODOM.ordered_fieldnames),))
        check('331 偏移：usec@0 / xyz@8 / q@20 / v@36 / ω@48',
              t_u == 1791389123456789
              and all(abs(a - b) < 1e-6 for a, b in zip(xyz, p_ev))
              and all(abs(a - b) < 1e-6 for a, b in zip(qq, _norm(q_ev))),
              'usec=%d xyz=%s' % (t_u, tuple(round(v, 4) for v in xyz)))
        # ★ 载荷里 q 的【线序】必须是 [w,x,y,z]（飞控按此读）。这条直接盯字节，
        # 旧码（把内部 (x,y,z,w) 原样塞进去）在这里 FAIL —— 就是它该抓的。
        _qw_exp = (_norm(q_ev)[3], _norm(q_ev)[0], _norm(q_ev)[1], _norm(q_ev)[2])
        check('331 q 载荷线序 = [w,x,y,z]（旧码在此 FAIL）',
              all(abs(a - b) < 1e-6 for a, b in zip(q_wire, _qw_exp)),
              'wire=%s 期望[w,x,y,z]=%s'
              % (tuple(round(v, 4) for v in q_wire), tuple(round(v, 4) for v in _qw_exp)))
        check('331 偏移：pose_cov@60（[0][6][11] 位置、[15][18][20] 姿态）',
              abs(pcov[0] - V.POS_VAR) < 1e-9 and abs(pcov[11] - V.POS_VAR) < 1e-9
              and abs(pcov[20] - V.POS_VAR) < 1e-9,
              '[0]=%.4f [6]=%.4f [11]=%.4f' % (pcov[0], pcov[6], pcov[11]))
        check('331 速度/角速度全是 NaN（本机不融合速度，见文件头）',
              all(math.isnan(v) for v in vvv) and all(math.isnan(v) for v in rpy_rate)
              and all(v == 0.0 for v in vcov),
              'v=%s ω=%s' % (tuple(round(v, 1) for v in vvv), tuple(round(v, 1) for v in rpy_rate)))
        check('331 尾部：frame_id@228 child_frame_id@229 reset_counter@230 '
              'estimator_type(VIO=3)@231 quality@232',
              (fid, cid, rst, est, qua) == (20, 20, 7, 3, 100),
              'got %s' % ((fid, cid, rst, est, qua),))
        check('331 frame_id = child_frame_id = LOCAL_FRD(20)',
              m.frame_id == 20 and m.child_frame_id == 20, '%s / %s' % (m.frame_id, m.child_frame_id))

    # PX4 侧（ev_control.cpp:118 + ev_pos_control.cpp:88）：q_error = q_ekf ⊗ q_ev⁻¹，
    # pos_ekf = Dcmf(q_error)·pos_ev。输入只用【报文里解出来的字节】+ 飞控自己的姿态，
    # 不用 u/p_ev/q_ev —— 这样验的是"PX4 拿到这串字节能不能还出真值"，不是自证。
    q_err = _norm(_qmul(Q_EKF_TRUE, _qinv(qq)))
    p_rec = _mv(_dcm(q_err), xyz)
    err_p = max(abs(a - b) for a, b in zip(p_rec, P_EKF_TRUE))
    check('%s 位置：PX4 还原 = 真北 NED（差 <1e-5 m）' % tag, err_p < 1e-5,
          '还原 %s 真值 %s' % (tuple(round(v, 4) for v in p_rec), P_EKF_TRUE))
    # 姿态同系检验：q_error ⊗ q_ev 必须等于飞控自己算的姿态
    q_chk = _norm(_qmul(q_err, qq))
    err_q = max(abs(abs(a) - abs(b)) for a, b in zip(q_chk, Q_EKF_TRUE))
    check('%s 姿态：q_error ⊗ q_ev = 飞控姿态（证明位置和姿态同系）' % tag, err_q < 1e-6,
          '差 %.2e' % err_q)

# --- 8b 运行路径：mode=3 不等 δ，且发的是【没补 δ】的原始值
u6 = mk_uplink(3)
for i in range(30):
    run_odom(u6, i, 90.0, None)          # 故意不给 ATTITUDE ⇒ δ 永远测不出来
check('mode=3 不需要 δ（没有 ATTITUDE 也照发）',
      u6.dyaw is None and len(u6.master.sent) == 30 and u6.n_no_dyaw == 0,
      'dyaw=%s 发=%d n_no_dyaw=%d' % (u6.dyaw, len(u6.master.sent), u6.n_no_dyaw))
m6 = u6.master.sent[-1]
check('mode=3 发的是原始视觉系位置（没被 δ 转过）',
      abs(m6.x - P_NED_V[0]) < 1e-9 and abs(m6.y - P_NED_V[1]) < 1e-9,
      'got (%.4f, %.4f) 期望 %s' % (m6.x, m6.y, P_NED_V[:2]))
check('mode=3 的 vx/vy/vz 是 NaN（PX4 那边 velocity_frame 保持 UNKNOWN）',
      all(math.isnan(v) for v in (m6.vx, m6.vy, m6.vz)),
      'got (%.1f, %.1f, %.1f)' % (m6.vx, m6.vy, m6.vz))
check('mode=3 发的四元数已归一化（EKF2 要求 1±1e-5，否则姿态被静默丢弃）',
      abs(math.sqrt(sum(c * c for c in m6.q)) - 1.0) < 1e-6,
      'norm=%.9f' % math.sqrt(sum(c * c for c in m6.q)))
# ψ_v=90° 且水平 ⇒ 机体在 NED/FRD 下就是纯 yaw 90°。期望用独立实现 _q_z 算，
# 不走 vision_uplink 的变换路径；跑的却是真实发送路径（run_odom → _process_odom
# → _make_odometry），所以这条直接锁住"报文里的四元数到底是哪个序"。
_q90 = _norm(_q_z(90.0))
_q90 = (_q90[3], _q90[0], _q90[1], _q90[2])            # (x,y,z,w) → [w,x,y,z]
check('mode=3 报文 q 线序 = [w,x,y,z]（真实发送路径；旧码在此 FAIL）',
      _same_rot(tuple(m6.q), _q90),
      'wire=%s 期望[w,x,y,z]=%s'
      % (tuple(round(v, 4) for v in m6.q), tuple(round(v, 4) for v in _q90)))
check('mode=3 与 mode=2 互斥（不会一帧发两种 pos_frame 把融合打断）',
      all(m.get_type() == 'ODOMETRY' for m in u6.master.sent),
      '类型：%s' % sorted({m.get_type() for m in u6.master.sent}))

# --- 8c 视觉世界系真跳变 ⇒ reset_counter +1（让 EKF 重置位置而不是当量测）
before = u6.odom_reset_n
LOGS[:] = []
run_odom(u6, 30, 90.0, None, p_enu=(P_ENU[0] + 5.0, P_ENU[1], P_ENU[2]))
check('odom 跳变 ⇒ reset_counter +1 且随下一帧发出去',
      u6.odom_reset_n == before + 1
      and u6.master.sent[-1].reset_counter == before + 1
      and any('reset_counter=' in s for s in LOGS),
      'reset_n=%d 报文里=%d %s'
      % (u6.odom_reset_n, u6.master.sent[-1].reset_counter,
         [s for s in LOGS if '跳变' in s][:1]))

print('\n' + '=' * 60)
if FAIL:
    print('✗ %d 项失败：%s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
if WARN:
    print('⚠ %d 项警告（不阻塞，但要知道）：%s' % (len(WARN), ', '.join(WARN)))
print('✓ 全部通过 —— 可以重启加载了')
