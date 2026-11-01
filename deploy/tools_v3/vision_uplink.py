#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VINS-Fusion → PX4 视觉注入 + 飞控遥测镜像（跑在 px4_imu_node 进程内）。

设计依据见同目录 VISION_UPLINK_CN.md。四条要点：

1) 为什么必须在本进程内：/dev/ttyACM0 只有一条，tty 的字节只给一个 reader。
   另起进程开同一个口，两边都会读到残缺字节流（2026-09-28 用 fc_ping 实测过：
   多读者 = 随机坏）。所以视觉上行和飞控遥测都搭在 px4_imu_node 的连接上。

2) 模式文件（默认 /root/xu_drone/live/vision_uplink.mode，每秒重读，热切换）：
       0 = 关（默认；连遥测请求都不发）
       1 = dry-run：算出要发的 NED 位姿，只写日志/CSV，不碰飞控
       2 = 真发：VISION_POSITION_ESTIMATE(102)，跟随 odom 频率（~15Hz）
       3 = 真发：ODOMETRY(331)，frame_id=LOCAL_FRD，送【原始视觉系】的位置+姿态
   改模式不用重启：echo 3 > vision_uplink.mode

3) 坐标（源码依据：VINS-Fusion utility/visualization.cpp pubOdometry）
       VINS world = 重力对齐、z 朝上、yaw 随初始化朝向（ENU 类）；body = FLU
       p_ned      = (y, x, -z)
       q_ned_frd  = Q_ENU2NED ⊗ q_world_body ⊗ Q_FLU2FRD
   ★ δ 对齐（2026-10-08 读 PX4 v1.17 源码后补）：mavlink_receiver.cpp:1321 把 102 号
     【硬编码】成 odom.pose_frame = POSE_FRAME_NED，位置不经任何旋转直接当真北 NED 用；
     而 VINS world 的 yaw 是"初始化那一刻机头指向"，两者差一个常值 δ。不补 δ 就发 =
     给 EKF 喂一个整体转过 δ 的位置（实测 δ≈−45°，在 6m 外就是 5m 的横偏）。
     补法：p_true = Rz(δ)·p，q_true = Qz(δ)⊗q，δ = FC_yaw − VINS_yaw（静止中位数）。
   ★ 102 号在本机【结构上不可能融合】（2026-10-08 读源码确认，实机 5998 帧全被忽略）：
     NED 分支要求 _control_status.flags.yaw_align（ev_pos_control.cpp:69）。能置
     yaw_align=true 的只有 4 处（全量 grep 过）：磁力计(mag_control.cpp:301)、GNSS
     (gps_control.cpp:532 / gnss_yaw_control.cpp:110)、EV yaw(ev_yaw_control.cpp:166)、
     外部航向注入(ekf.h:429 resetHeadingToExternalObservation)。本机 CAL_MAG0/1_ID=0
     （无磁）、无 GPS、EKF2_EV_CTRL=3（yaw 位没开）⇒ yaw_align 恒为假 ⇒ 位置融合
     静默拒绝（stopEvPosFusion 在 ev_pos 为假时是空操作），没有任何 MAVLink 诊断。
   ★ 改用 331 号 ODOMETRY + MAV_FRAME_LOCAL_FRD（mode=3）绕开这个死结：
     331 的 frame_id 由发送方决定（mavlink_receiver.cpp:1372 MAV_FRAME_LOCAL_FRD →
     POSE_FRAME_FRD），而 ev_pos_control.cpp:88 的 FRD 分支【没有】yaw_align 要求，
     它用 R_ev_to_ekf = _ev_q_error_filt（ev_control.cpp:65 每帧无条件更新，首个样本
     直接 reset 成精确值）把位置转进 EKF 自己的 NED。⇒ 只要位置和姿态是【同一个系】
     里的量就对。所以 mode=3 送【不补 δ】的 ned/q_ned_frd（VINS 世界系，NED 轴约定），
     补了 δ 反而错——那会让 EKF 少转一次。EKF 若发现系不对，会自己从姿态里量出来。
   ★ 线序（2026-10-09 定位的 bug）：331 的 q[4] 由 MAVLink 规定为 **[w,x,y,z]**
     （pymavlink 字段文档原文 "Quaternion components, w, x, y, z"），而本模块内部
     一律 (x,y,z,w)。曾经把内部序原样塞进报文 ⇒ 飞控读到乱序四元数 ⇒ R_ev_to_ekf
     学成错旋转 ⇒ EKF 位置被整体转错。这类错【误差 ∝ 距视觉原点距离】：在原点附近
     两种模型重合，所以当年在原点做的"跟随判据"是假阳性。_make_odometry 里换序。
   ★ 更正（2026-10-08）：ECL_INFO/ECL_WARN 在 EKF2 里是 printf/fprintf(stderr)
     （EKF/estimator_interface.h:52），【不产生 STATUSTEXT】。所以"没看到 [FC] 输出"
     不能推出"融合没启动"。飞控侧唯一 MAVLink 可见的融合信号是 LOCAL_POSITION_NED
     本身：位置融合启动时无 GNSS 会走 startEvPosFusion 的 resetHorizontalPositionTo
     （ev_pos_control.cpp:225）⇒ EKF 的水平位置会【跳到】我们送的值（台架上从
     (3.8,6.7) 跳到 ≈(0,0)）。那一下就是"数据到了且融合起来了"的铁证。
   102 号消息 PX4 用【欧拉角】（源码：Eulerf(vpe.roll,pitch,yaw)），所以发 rpy。
   ★ 实测（2026-10-08）：现行 MAVLink 定义里 covariance 是【扩展字段】——
     7 个基础字段算出的 crc_extra=158（与 pymavlink 实测值吻合），加上 covariance
     当基础字段算则是 45。MAVLink 1 没有扩展字段 ⇒ v1 报文里 covariance 会被
     PX4 补零 ⇒ 若 EKF2_EV_NOISE_MD=1（信消息方差）就成了"零方差=无限自信"。
     所以本模块强制 MAVLINK20=1，发 v2 帧，载荷 116 字节（32 基础+84 协方差）。

4) 时间戳：time_usec 必须是【飞控时间基】（PX4 会把它过 sync_stamp()，
   而本工程从没有 TIMESYNC 交换 → offset 恒 0 → 原样进 EKF2）。
   反变换用最近一条 HIGHRES_IMU 的 (采样时刻 fc, 映射后宿主时刻 host) 做局部
   线性换算，scale 用 ImuClockMapper 的 ppm。这样视觉和 IMU 用的是同一条
   时钟关系，天然自洽。

★ 本模块所有函数都跑在 IMU 主循环线程里，任何一处阻塞都会在 /imu 上凿洞
  （2026-09-26 那次 500ms 洞把 VIO 毁掉的教训）。规矩：
    · 不发 sleep（参数逐个发，一次 tick 发一条）
    · 不做大文件 I/O（CSV 都是单行 append）
    · 逻辑尽量 O(1)
"""

import collections
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# 必须在 import pymavlink 之前 —— 102 号的 covariance 是扩展字段，v1 带不出去（详见文件头）
os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil  # noqa: E402

# ---------------------------------------------------------------- 配置
ODOM_TOPIC = os.environ.get('VISION_ODOM_TOPIC', '/vins_estimator/odometry')
MODE_FILE = os.environ.get('VISION_MODE_FILE', '/root/xu_drone/live/vision_uplink.mode')
CSV_DIR = os.environ.get('VISION_CSV_DIR', '/tmp/vins_pi')

# 发给 PX4 的协方差。PX4 只读 6 个对角项：位置 [0][6][11]、姿态 [15][18][20]。
# 只在 EKF2_EV_NOISE_MD=1（信消息不信参数）时起作用。
# 默认 0.04 m²（σ=20cm）/ 0.03 rad² —— 宁可保守，别在飞控面前吹牛。
def _env_var(name, default):
    """读方差：<= 0 一律不接受。方差 0 = "无限自信"，比不传更糟；
    而且全零协方差会被 MAVLink 2 当尾部默认值裁掉，PX4 那边还是读到零。"""
    try:
        v = float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)
    return v if v > 0.0 else float(default)


POS_VAR = _env_var('VISION_POS_VAR', '0.04')
ANG_VAR = _env_var('VISION_ANG_VAR', '0.03')

# 安全闸：解算出来的位置超过这个量级（或非有限值）就不发 —— 宁可停发也不喂坏数
MAX_ABS_M = float(os.environ.get('VISION_MAX_M', '50.0'))
# 时钟参考对超过这么久没更新（IMU 桥断了/卡了）就不发
REF_STALE_S = float(os.environ.get('VISION_REF_STALE_S', '2.0'))
# odom 时间戳比"现在"旧这么多就不发（VINS 卡了/队列积压）
MAX_LATENCY_S = float(os.environ.get('VISION_MAX_LATENCY_S', '0.5'))

# --- δ 对齐（视觉世界系 ↔ 飞控真北 NED）------------------------------------
# δ = FC_yaw − VINS_yaw，常值。只在"两路姿态配对到同一物理时刻"时采样：
# ATTITUDE.time_boot_ms（飞控时间基）对上我们反算的 fc_usec。这样即使机体在转，
# 配对也不被两路延迟差污染（按到达时刻配对的话，30°/s × 100ms = 3° 误差）。
# 采样够多且离散度小才认；δ 无效时 mode=2 宁可不发（发出去就是整体转过的位置）。
DYAW_MIN_N = int(os.environ.get('VISION_DYAW_MIN_N', '30'))          # 至少这么多样本
DYAW_PAIR_MS = float(os.environ.get('VISION_DYAW_PAIR_MS', '150'))   # 配对允许的时间差
DYAW_MAX_SPREAD_DEG = float(os.environ.get('VISION_DYAW_MAX_SPREAD_DEG', '3.0'))
DYAW_ATT_MAX_AGE_S = float(os.environ.get('VISION_DYAW_ATT_MAX_AGE_S', '0.5'))
DYAW_FC_JUMP_DEG = float(os.environ.get('VISION_DYAW_FC_JUMP_DEG', '20.0'))  # 飞控 yaw 跳变
DYAW_ODOM_JUMP_M = float(os.environ.get('VISION_DYAW_ODOM_JUMP_M', '0.5'))   # odom 位置跳变

MODE_DESC = {0: '关', 1: 'dry-run（只算不发）',
             2: '真发 102(VPE/NED)', 3: '真发 331(ODOMETRY/FRD 原始视觉系)'}

# 启动时要 dump 的飞控参数（此固件没有的名字飞控不回复，不会报错）
PARAM_WATCH = [
    'EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD',
    'EKF2_EVP_NOISE', 'EKF2_EVV_NOISE', 'EKF2_EVA_NOISE',
    'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
    'EKF2_EV_QMIN', 'EKF2_EVP_GATE', 'EKF2_EVV_GATE', 'EKF2_EV_GATE',
    'EKF2_HGT_REF', 'EKF2_HGT_MODE', 'EKF2_AID_MASK',
    'EKF2_GPS_CTRL', 'EKF2_BARO_CTRL', 'EKF2_MAG_TYPE', 'EKF2_MAG_CHECK',
    # 磁力计：PX4 的 yaw_align 得靠磁（EKF2_EV_CTRL 的 yaw 位没开），没磁就没有
    # yaw_align → NED 分支的位置融合被静默拒绝（ev_pos_control.cpp:80）
    'CAL_MAG0_ID', 'CAL_MAG0_PRIO', 'CAL_MAG1_ID',
    'COM_ARM_EKF_POS', 'COM_ARM_EKF_VEL', 'COM_ARM_EKF_HGT', 'COM_ARM_EKF_YAW',
    'COM_RC_IN_MODE', 'NAV_RCL_ACT', 'COM_OF_LOSS_T',
    'MAV_1_CONFIG', 'MAV_2_CONFIG', 'MAV_PROTO_VER', 'MAV_TYPE', 'SYS_AUTOSTART',
    'CBRK_FLIGHTTERM', 'MPC_THR_HOVER',
]

# PX4 estimator_status.flags 位义（estimator_status.msg）
EST_FLAGS = [
    (0, 'attitude'), (1, 'vel_h'), (2, 'vel_v'), (3, 'pos_h_rel'), (4, 'pos_h_abs'),
    (5, 'pos_v_abs'), (6, 'pos_v_agl'), (7, 'const_pos'), (8, 'pred_pos_h_rel'),
    (9, 'pred_pos_h_abs'), (10, 'gps_glitch'), (11, 'accel_err'),
]

# 4x4 协方差上三角展平后，我们填的那 6 个对角项的下标
COV_XX, COV_YY, COV_ZZ, COV_RR, COV_PP, COV_YAW = 0, 6, 11, 15, 18, 20

# 331 号 ODOMETRY：frame_id 由发送方决定（这是绕开 yaw_align 死结的关键，见文件头 ★）。
# getattr 取，躲开 pymavlink 版本的命名差异；取不到就用 MAVLink 协议里的定值。
MAV_FRAME_LOCAL_FRD = getattr(mavutil.mavlink, 'MAV_FRAME_LOCAL_FRD', 20)
MAV_ESTIMATOR_TYPE_VIO = getattr(mavutil.mavlink, 'MAV_ESTIMATOR_TYPE_VIO', 3)


def _sensor_bits():
    """SYS_STATUS 里我们关心的位（用 getattr 取，躲开 pymavlink 版本的命名差异）。"""
    out = []
    for nm, label in (
            ('MAV_SYS_STATUS_SENSOR_3D_GYRO', '陀螺'),
            ('MAV_SYS_STATUS_SENSOR_3D_ACCEL', '加计'),
            ('MAV_SYS_STATUS_SENSOR_3D_MAG', '磁力计'),
            ('MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE', '气压'),
            ('MAV_SYS_STATUS_SENSOR_GPS', 'GPS'),
            ('MAV_SYS_STATUS_SENSOR_VISION_POSITION', '视觉定位'),
            ('MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW', '光流')):
        v = getattr(mavutil.mavlink, nm, None)
        if isinstance(v, int):
            out.append((v, label))
    return out


# SYS_STATUS 直接回答"到底有没有磁力计、健康不健康" —— yaw_align 的前提
SYS_SENSOR_BITS = _sensor_bits()

# ENU→NED：绕 (1,1,0)/√2 转 180°；FLU→FRD：绕 x 轴转 180°（四元数 x,y,z,w）
Q_ENU2NED = (0.7071067811865476, 0.7071067811865476, 0.0, 0.0)
Q_FLU2FRD = (1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------- 小工具
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def wrap_pi(x):
    """角度规整到 (−π, π]。"""
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def q_to_rpy(q):
    """四元数 (x,y,z,w) → (roll, pitch, yaw)，ZYX 约定，弧度。"""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_q(r, p, y):
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class VisionUplink(object):
    """挂在 px4_imu_node 上的视觉上行 + 遥测镜像。

    所有串口 I/O（发送 / 参数请求）都只在 tick() 里做，由 IMU 主循环单线程调用；
    本类自己不开线程，也不碰 master.recv_*，和 IMU 的 recv_match 不会打架。
    """

    EXTRA_TYPES = ('ESTIMATOR_STATUS', 'LOCAL_POSITION_NED', 'ATTITUDE',
                   'PARAM_VALUE', 'AUTOPILOT_VERSION',
                   # STATUSTEXT = 飞控自己的控制台（ECL_INFO/ECL_WARN）。这是判断
                   # "视觉数据到底进没进 EKF2"的唯一直接手段，详见文件头。
                   'STATUSTEXT', 'SYS_STATUS', 'HEARTBEAT', 'COMMAND_ACK')

    # PX4 控制台里该高亮的关键词（直接回答诊断问题的那几条）
    FC_KEYWORDS = ('ev position', 'ev height', 'vision', 'yaw align',
                   'magnetometer', 'mag ')
    SEVERITY = ('EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR',
                'WARNING', 'NOTICE', 'INFO', 'DEBUG')

    def __init__(self):
        self.master = None
        self.mode = 0
        self._t_mode = 0.0
        self._t_status = 0.0
        self._started = False

        # 最近一条 odometry。rospy 回调只做"存一份"；真正的发送在主循环里做。
        self.odom = None            # (msg, arrival_wall, seq)
        self.sent_seq = 0

        # 统计
        self.n_sent = 0
        self.n_skip_bad = 0
        self.n_skip_late = 0
        self.n_no_ref = 0
        self.n_stale_ref = 0
        self.t_first = time.time()
        self.last_latency = float('nan')

        # 飞控遥测镜像
        self.ekf_ned = None         # (x,y,z,vx,vy,vz) 来自 LOCAL_POSITION_NED
        self.att_rpy = None         # (r,p,y) 弧度，来自 ATTITUDE
        self.att_t = 0.0            # 收到 ATTITUDE 的墙钟时刻
        self.att_tb_ms = 0          # ATTITUDE.time_boot_ms（飞控时间基，配对 δ 用）
        self.est_flags = 0
        self.fc_version = None

        # 飞控健康 / 状态（回答"有没有磁力计、解锁没解锁"）
        self.armed = None           # None=还没收到 HEARTBEAT
        self.sys_status = None
        self._last_sys_line = None
        self._last_stext = ''
        self._last_stext_n = 0
        self.n_statustext = 0

        # δ 对齐
        self.dyaw = None            # δ（弧度）；None = 还没测出来 → mode=2 不发
        self._dyaw_buf = collections.deque(
            maxlen=int(os.environ.get('VISION_DYAW_BUF', '240')))
        self._psi_fc_prev = None    # (t, yaw) 飞控 yaw 跳变检测
        self._pos_prev = None       # (t, x, y, z) odom 跳变检测（VINS 重开/回环）
        self.n_no_dyaw = 0

        # 331 的 reset_counter：视觉世界系一跳（VINS 重开/回环）就 +1，
        # 告诉 EKF"我的参考系换了，请重置水平位置"（ev_reset → resetHorizontalPositionTo）
        self.odom_reset_n = 0

        # 状态行用：最后一次"新鲜帧"通过的墙钟时刻（判有没有在停发）
        self._t_fresh = 0.0
        self._t_late_log = 0.0

        # 1Hz 状态行
        self._pose = None           # mode2 发的：(ned_sent 已补δ, (r,p,y)_sent, latency)
        self._pose3 = None          # mode3 发的：(ned 原始视觉系, (r,p,y) 原始, latency)
        self._n_sent_win = 0
        self._t_win = 0.0

        # 参数 dump
        self.params = {}
        self._param_queue = []       # 还没请求的
        self._param_pending = set()  # 请求了还没回的
        self._param_deadline = 0.0
        self._param_dumped = False

        # CSV
        self.csv_vision = None
        self.csv_telem = None
        self._open_csv()

        # dialect 自检：没有 covariance 字段说明退回 MAVLink 1 了，得先说清楚风险
        try:
            self.vpe_fields = len(
                mavutil.mavlink.MAVLink_vision_position_estimate_message.ordered_fieldnames)
        except Exception:  # noqa: BLE001
            self.vpe_fields = 0
        if self.vpe_fields < 8:
            rospy.logwarn('[vision] ⚠ 当前 dialect 的 102 号只有 %d 个字段（无 covariance，'
                          'MAVLink %s）—— 飞控若 EKF2_EV_NOISE_MD=1 会把零方差当"无限自信"。'
                          '请确认 MAVLINK20=1 且在 import pymavlink 之前设置',
                          self.vpe_fields, mavutil.mavlink.WIRE_PROTOCOL_VERSION)
        else:
            rospy.loginfo('[vision] MAVLink %s，102 号 %d 字段（含 covariance，crc_extra=%d）',
                          mavutil.mavlink.WIRE_PROTOCOL_VERSION, self.vpe_fields,
                          mavutil.mavlink.MAVLink_vision_position_estimate_message.crc_extra)

        rospy.Subscriber(ODOM_TOPIC, Odometry, self._on_odom, queue_size=50)
        self.pub_ned = rospy.Publisher('/px4/local_position_ned', PoseStamped, queue_size=20)
        self.pub_status = rospy.Publisher('/px4/estimator_status', String, queue_size=20)
        self.pub_stext = rospy.Publisher('/px4/statustext', String, queue_size=20)
        rospy.loginfo('[vision] 已挂载：订阅 %s，镜像话题 /px4/local_position_ned、'
                      '/px4/estimator_status、/px4/statustext；模式文件 %s',
                      ODOM_TOPIC, MODE_FILE)

    # ------------------------------------------------------------ CSV
    # vins_* = 原始 odom（ENU/FLU）；frd_* / qfrd_* = mode3 实际发出去的
    # （原始视觉系、未补 δ）；ned_* / roll..yaw_* = mode2 实际发出去的（已补 δ）
    CSV_VISION_HEAD = ('wall,odom_stamp,latency_s,mode,fc_usec,'
                       'vins_x,vins_y,vins_z,ned_x,ned_y,ned_z,'
                       'roll_deg,pitch_deg,yaw_deg,dyaw_deg,'
                       'frd_x,frd_y,frd_z,qfrd_r_deg,qfrd_p_deg,qfrd_y_deg,reset_n,'
                       'ekf_x,ekf_y,ekf_z,flags\n')
    CSV_TELEM_HEAD = ('wall,x,y,z,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,'
                      'flags,pos_h_ratio,pos_v_ratio,vel_ratio\n')

    def _open_append(self, name, head):
        """append 打开；表头对不上就先把旧文件挪成 .old（改列不能混表头）。"""
        p = os.path.join(CSV_DIR, name)
        if os.path.exists(p):
            try:
                with open(p) as f:
                    same = (f.readline() == head)
            except Exception:  # noqa: BLE001
                same = False
            if not same:
                try:
                    os.replace(p, p + '.old')
                    rospy.loginfo('[vision] %s 列变了，旧文件存为 %s.old', name, name)
                except Exception:  # noqa: BLE001
                    pass
        f = open(p, 'a', buffering=1)
        if os.path.getsize(p) == 0:
            f.write(head)
        return f

    def _open_csv(self):
        try:
            if not os.path.isdir(CSV_DIR):
                os.makedirs(CSV_DIR)
            self.csv_vision = self._open_append('vision_uplink.csv', self.CSV_VISION_HEAD)
            self.csv_telem = self._open_append('px4_telemetry.csv', self.CSV_TELEM_HEAD)
        except Exception as e:  # noqa: BLE001
            self.csv_vision = None
            self.csv_telem = None
            rospy.logwarn('[vision] CSV 打不开（不影响发送）: %s', e)

    # ------------------------------------------------------------ 模式
    def _read_mode_file(self):
        try:
            with open(MODE_FILE) as f:
                v = f.read().strip().split()
            m = int(v[0]) if v else 0
            return m if m in (0, 1, 2, 3) else 0
        except Exception:  # noqa: BLE001
            return 0

    def extra_types(self):
        """给 recv_match 用的额外消息类型。mode=0 时不给，保证零额外流量。"""
        return list(self.EXTRA_TYPES) if self.mode > 0 else []

    def on_reconnect(self):
        """串口重连后，握手要重来一遍；δ 也作废（飞控可能重启过，yaw 基准未必还是原来那个）。"""
        self._started = False
        self._param_dumped = False
        self._param_queue = list(PARAM_WATCH)
        self._param_pending = set()
        self._dyaw_buf.clear()
        self.dyaw = None
        self._psi_fc_prev = None
        self._pos_prev = None

    def _on_odom(self, msg):
        # rospy 每个订阅者一个回调线程 → 这里不会重入，seq 自增是安全的
        n = self.odom[2] + 1 if self.odom is not None else 1
        self.odom = (msg, time.time(), n)

    # ------------------------------------------------------------ 启动握手（不阻塞）
    def _start(self, master):
        self.master = master
        tgt_sys, tgt_comp = master.target_system, master.target_component
        try:
            for mid, interval in (
                    (mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 200000),   # 5Hz
                    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000),  # 10Hz
                    (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000),            # 10Hz
                    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000)):        # 1Hz
                master.mav.command_long_send(
                    tgt_sys, tgt_comp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, mid, interval, 0, 0, 0, 0, 0)
            master.mav.command_long_send(
                tgt_sys, tgt_comp, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
            self._param_queue = list(PARAM_WATCH)
            self._param_pending = set()
            rospy.loginfo('[vision] 已请求遥测流：ESTIMATOR_STATUS 5Hz / '
                          'LOCAL_POSITION_NED 10Hz / ATTITUDE 10Hz / SYS_STATUS 1Hz；'
                          '参数逐个请求中（%d 个，每次主循环一条，不阻塞）',
                          len(self._param_queue))
            rospy.loginfo('[vision] 等两样东西：① 飞控控制台（STATUSTEXT）'
                          '② δ 对齐样本（静止约 5 秒收敛）。δ 无效时 mode=2 不会发。')
            self._started = True
        except Exception as e:  # noqa: BLE001
            rospy.logwarn('[vision] 启动握手失败（下一轮重试）: %s', e)
            self._started = False

    # ------------------------------------------------------------ 收遥测
    def on_mavlink(self, msg):
        t = msg.get_type()
        if t == 'LOCAL_POSITION_NED':
            self.ekf_ned = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            self._pub_ned()
        elif t == 'ATTITUDE':
            self.att_rpy = (msg.roll, msg.pitch, msg.yaw)
            self.att_t = time.time()
            self.att_tb_ms = getattr(msg, 'time_boot_ms', 0)
            self._pub_ned()
        elif t == 'ESTIMATOR_STATUS':
            self._on_est_status(msg)
        elif t == 'STATUSTEXT':
            self._on_statustext(msg)
        elif t == 'SYS_STATUS':
            self._on_sys_status(msg)
        elif t == 'HEARTBEAT':
            self._on_heartbeat(msg)
        elif t == 'COMMAND_ACK':
            rospy.loginfo('[vision][FC] COMMAND_ACK cmd=%s result=%s',
                          msg.command, getattr(msg, 'result', '?'))
        elif t == 'PARAM_VALUE':
            self._on_param(msg)
        elif t == 'AUTOPILOT_VERSION':
            v = msg.flight_sw_version
            self.fc_version = '%d.%d.%d' % ((v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF)
            rospy.loginfo('[vision] 飞控固件: PX4 v%s (vendor %d, product %d)',
                          self.fc_version, msg.vendor_id, msg.product_id)

    def _on_param(self, msg):
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('utf-8', 'ignore')
        name = name.rstrip('\x00')
        if name not in self._param_pending:
            return
        val, ptype = msg.param_value, msg.param_type
        if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            if val != 0.0 and abs(val) < 1e-30:
                # PX4 有时候把 int 参数按 float 位模式塞进来（读出来是 denormal）
                import struct
                val = struct.unpack('<i', struct.pack('<f', val))[0]
            else:
                val = int(round(val))
        self.params[name] = (val, ptype)
        self._param_pending.discard(name)
        if not self._param_pending and not self._param_queue:
            self._write_params()

    def _write_params(self):
        if self._param_dumped:
            return
        self._param_dumped = True
        try:
            p = os.path.join(CSV_DIR, 'px4_params.csv')
            with open(p, 'w') as f:
                f.write('name,value,type\n')
                for k in PARAM_WATCH:
                    if k in self.params:
                        v, t = self.params[k]
                        f.write('%s,%s,%d\n' % (k, v, t))
        except Exception:  # noqa: BLE001
            pass
        rospy.loginfo('[vision] === 飞控参数 dump（%d/%d，完整表 %s/px4_params.csv）===',
                      len(self.params), len(PARAM_WATCH), CSV_DIR)
        for k in PARAM_WATCH:
            if k not in self.params:
                rospy.loginfo('[vision]   %-18s = （此固件没有）', k)
                continue
            v = self.params[k][0]
            hint = ''
            if k == 'EKF2_EV_CTRL':
                bits = [nm for b, nm in ((0, '水平位置'), (1, '垂直位置'),
                                         (2, '速度'), (3, 'yaw')) if int(v) & (1 << b)]
                hint = '  ← 视觉融合开关: [%s]' % ('+'.join(bits) if bits else '全关')
            elif k == 'EKF2_HGT_REF':
                hint = '  ← 0=气压 1=GPS 2=测距 3=视觉'
            elif k == 'EKF2_AID_MASK':
                hint = '  ← 老固件视觉位: bit3=位置 bit4=yaw'
            rospy.loginfo('[vision]   %-18s = %s%s', k, v, hint)

    def _on_est_status(self, msg):
        self.est_flags = msg.flags
        names = [nm for b, nm in EST_FLAGS if msg.flags & (1 << b)]
        self.pub_status.publish(String(data=(
            'flags=0x%x [%s] pos_h_ratio=%.3f pos_v_ratio=%.3f vel_ratio=%.3f '
            'pos_h_acc=%.2f pos_v_acc=%.2f'
            % (msg.flags, ','.join(names), msg.pos_horiz_ratio, msg.pos_vert_ratio,
               msg.vel_ratio, msg.pos_horiz_accuracy, msg.pos_vert_accuracy))))
        if self.csv_telem is not None:
            try:
                ex = self.ekf_ned or (float('nan'),) * 6
                r, p, y = self.att_rpy or (float('nan'),) * 3
                self.csv_telem.write(
                    '%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,0x%x,%.3f,%.3f,%.3f\n'
                    % (time.time(), ex[0], ex[1], ex[2], ex[3], ex[4], ex[5],
                       math.degrees(r), math.degrees(p), math.degrees(y),
                       msg.flags, msg.pos_horiz_ratio, msg.pos_vert_ratio, msg.vel_ratio))
            except Exception:  # noqa: BLE001
                pass

    def _pub_ned(self):
        if self.ekf_ned is None:
            return
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = 'px4_ned'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.ekf_ned[0:3]
        if self.att_rpy is not None:
            qx, qy, qz, qw = rpy_to_q(*self.att_rpy)
            m.pose.orientation.x, m.pose.orientation.y = qx, qy
            m.pose.orientation.z, m.pose.orientation.w = qz, qw
        self.pub_ned.publish(m)

    # ------------------------------------------------------------ 飞控控制台 / 健康
    def _on_statustext(self, msg):
        """镜像 PX4 控制台（ECL_INFO/ECL_WARN 都从这儿出来）。

        ★ 这是判断"视觉数据到底进没进 EKF2"的唯一直接手段：
          · "starting EV height fusion"           ⇒ 数据到了（高度融合不要求 yaw_align）
          · "starting EV position fusion, resetting state" ⇒ 水平位置融合也起来了
          · "stopping EV position fusion, continuing conditions failing" ⇒ 被条件挡下
          · 一句都没有                              ⇒ 数据压根没到 EKF2
        （被 yaw_align 挡住时 PX4 是【静默】的：ev_pos 还是假，stopEvPosFusion() 空转。）
        """
        txt = msg.text
        if isinstance(txt, bytes):
            txt = txt.decode('utf-8', 'ignore')
        txt = txt.rstrip('\x00').strip()
        if not txt:
            return
        self.n_statustext += 1
        if txt == self._last_stext:
            self._last_stext_n += 1
            if self._last_stext_n % 20 != 1:
                return                       # 同一条刷屏：20 条只打 1 条
        else:
            self._last_stext = txt
            self._last_stext_n = 1
        sev = self.SEVERITY[msg.severity] if 0 <= msg.severity < len(self.SEVERITY) \
            else str(msg.severity)
        low = txt.lower()
        mark = '★ ' if any(k in low for k in self.FC_KEYWORDS) else ''
        rospy.loginfo('[vision][FC] %s%s: %s', mark, sev, txt)
        try:
            self.pub_stext.publish(String(data='%s: %s' % (sev, txt)))
        except Exception:  # noqa: BLE001
            pass

    def _on_sys_status(self, msg):
        """SYS_STATUS 直接回答"有没有磁力计、健不健康"——yaw_align 的前提条件。"""
        self.sys_status = msg
        parts = []
        for bit, label in SYS_SENSOR_BITS:
            if not (msg.onboard_control_sensors_present & bit):
                st = '无'
            elif not (msg.onboard_control_sensors_enabled & bit):
                st = '未启用'
            else:
                st = '健康' if (msg.onboard_control_sensors_health & bit) else '★不健康'
            parts.append('%s=%s' % (label, st))
        line = ' '.join(parts)
        if line != self._last_sys_line:
            self._last_sys_line = line
            rospy.loginfo('[vision][FC] SYS_STATUS: %s', line)

    def _on_heartbeat(self, msg):
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed != self.armed:
            self.armed = armed
            rospy.logwarn('[vision][FC] 飞控%s（本模块只发视觉，从不发解锁/起飞指令）',
                          '★已解锁★' if armed else '已上锁')

    # ------------------------------------------------------------ δ 对齐
    def _sample_dyaw(self, now, fc_usec, yaw_v):
        """采一个 δ = FC_yaw − VINS_yaw 样本；够了就更新。

        配对条件：ATTITUDE.time_boot_ms ≈ 我们反算的 fc_usec（同一物理时刻）——
        这样机体在转也不会引入 ω·Δt 的偏差。飞控 yaw 一次跳 > DYAW_FC_JUMP_DEG
        （磁重对齐/飞控重启）就把已采的全作废重来。
        """
        if self.att_rpy is None or self.att_tb_ms <= 0:
            return                       # 还没收到带时间戳的 ATTITUDE
        if (now - self.att_t) > DYAW_ATT_MAX_AGE_S:
            return                       # ATTITUDE 太旧（流断了）
        psi_fc = self.att_rpy[2]
        prev = self._psi_fc_prev
        self._psi_fc_prev = (now, psi_fc)
        if prev is not None and abs(wrap_pi(psi_fc - prev[1])) > math.radians(DYAW_FC_JUMP_DEG):
            self._dyaw_buf.clear()
            self.dyaw = None
            rospy.logwarn('[vision] 飞控 yaw 跳变 %.1f°（磁重对齐/重启？）→ δ 作废重测',
                          math.degrees(wrap_pi(psi_fc - prev[1])))
            return
        if abs(self.att_tb_ms * 1000 - fc_usec) > DYAW_PAIR_MS * 1000.0:
            return                       # 两路不是同一时刻，不配
        self._dyaw_buf.append(wrap_pi(psi_fc - yaw_v))
        if len(self._dyaw_buf) < DYAW_MIN_N:
            return
        ds = sorted(self._dyaw_buf)
        ref = ds[len(ds) // 2]                       # 中位数抗离群
        dev = [wrap_pi(x - ref) for x in ds]
        spread = max(dev) - min(dev)
        if spread > math.radians(DYAW_MAX_SPREAD_DEG):
            return                       # 这批不干净就沿用上一次的 δ（δ 是常值，不乱动）
        v = wrap_pi(ref + sum(dev) / len(dev))       # 簇内再取圆均值，比中位数更准
        if self.dyaw is None:
            rospy.loginfo('[vision] δ 对齐完成：%.2f°（%d 样本，离散度 %.2f°）'
                          '—— 位置/姿态都按它转到飞控真北',
                          math.degrees(v), len(ds), math.degrees(spread))
        self.dyaw = v

    # ------------------------------------------------------------ 主循环每一跳
    def tick(self, master, ref, now):
        # 模式热切换（每秒重读一次文件）
        if now - self._t_mode >= 1.0:
            self._t_mode = now
            m = self._read_mode_file()
            if m != self.mode:
                rospy.loginfo('[vision] 模式 %d(%s) → %d(%s)',
                              self.mode, MODE_DESC.get(self.mode, '?'),
                              m, MODE_DESC.get(m, '?'))
                self.mode = m
                if m > 0:
                    self._started = False    # 重新握手（请求遥测流 + dump 参数）

        if self.mode <= 0 or master is None:
            return

        if not self._started:
            self._start(master)
            return

        # 参数：一次 tick 发一条，绝不 sleep（这是 200Hz 的 IMU 主循环）
        if self._param_queue:
            name = self._param_queue.pop(0)
            try:
                master.mav.param_request_read_send(
                    master.target_system, master.target_component,
                    name.encode('utf-8'), -1)
                self._param_pending.add(name)
            except Exception:  # noqa: BLE001
                pass
            if not self._param_queue:
                self._param_deadline = now + 5.0     # 全发完了，等 5 秒收回复
            return
        if self._param_pending and now > self._param_deadline:
            self._write_params()

        self._process_odom(ref, now)
        self._status_line(now)

    # ------------------------------------------------------------ 处理一帧 odom
    def _process_odom(self, ref, now):
        odom = self.odom
        if odom is None or odom[2] == self.sent_seq:
            return                      # 还没有新 odom
        msg, _arrival, seq = odom
        self.sent_seq = seq

        stamp = msg.header.stamp.to_sec()
        latency = now - stamp
        self.last_latency = latency
        if latency > MAX_LATENCY_S or latency < -MAX_LATENCY_S:
            self.n_skip_late += 1
            # 每 5 秒报一次（%20 是按帧数，15Hz 下 1.3 秒一次，太吵；用时间更好）
            if now - self._t_late_log > 5.0:
                self._t_late_log = now
                rospy.logwarn('[vision] odom 时间戳偏差 %.0fms，本轮不发（VINS 卡了？'
                              '已停发 %.0fs，累计跳过 %d 帧）',
                              latency * 1e3, now - self._t_fresh, self.n_skip_late)
            return
        self._t_fresh = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # ENU/FLU → NED/FRD
        ned = (p.y, p.x, -p.z)
        q_ned_frd = qmul(qmul(Q_ENU2NED, (q.x, q.y, q.z, q.w)), Q_FLU2FRD)
        roll, pitch, yaw = q_to_rpy(q_ned_frd)

        # 安全闸
        if not all(map(math.isfinite, ned)) or max(abs(v) for v in ned) > MAX_ABS_M:
            self.n_skip_bad += 1
            if self.n_skip_bad % 20 == 1:
                rospy.logwarn('[vision] 位置不合理，跳过 (ned=%.2f,%.2f,%.2f)',
                              ned[0], ned[1], ned[2])
            return

        # odom 位置突跳 → VINS 重开 / 回环跳变：世界系换了，δ 必须重测
        prev = self._pos_prev
        self._pos_prev = (now, ned[0], ned[1], ned[2])
        if prev is not None and (now - prev[0]) < 0.5:
            jump = math.sqrt(sum((ned[i] - prev[i + 1]) ** 2 for i in range(3)))
            if jump > DYAW_ODOM_JUMP_M:
                self._dyaw_buf.clear()
                self.dyaw = None
                self.odom_reset_n = (self.odom_reset_n + 1) % 256
                rospy.logwarn('[vision] odom 位置跳变 %.2fm/%.0fms（VINS 重开或回环？）'
                              '→ δ 作废重测，reset_counter=%d',
                              jump, (now - prev[0]) * 1e3, self.odom_reset_n)

        # 时间戳：宿主 epoch → 飞控时间基（用最近一对 IMU 时钟参考做局部换算）
        if ref is None:
            self.n_no_ref += 1
            return
        fc_ref, host_ref, scale_ppm = ref
        if host_ref is None or (now - host_ref) > REF_STALE_S:
            self.n_stale_ref += 1
            if self.n_stale_ref % 20 == 1:
                rospy.logwarn('[vision] IMU 时钟参考过旧，本轮不发')
            return
        fc_usec = int(fc_ref + (stamp - host_ref) * 1e6 * (1.0 - scale_ppm * 1e-6))

        # δ 采样：mode=1 的主要用途就是先把 δ 测出来
        self._sample_dyaw(now, fc_usec, yaw)

        # 补 δ：VINS 世界系 → 飞控真北 NED
        # （102 号被 PX4 硬编码成 POSE_FRAME_NED，位置不经旋转直接当真北用，见文件头）
        if self.dyaw is None:
            ned_s, rpy_s = ned, (roll, pitch, yaw)
        else:
            cd, sd = math.cos(self.dyaw), math.sin(self.dyaw)
            ned_s = (cd * ned[0] - sd * ned[1],
                     sd * ned[0] + cd * ned[1],
                     ned[2])
            h = self.dyaw * 0.5
            rpy_s = q_to_rpy(qmul((0.0, 0.0, math.sin(h), math.cos(h)), q_ned_frd))
        self._pose = (ned_s, rpy_s, latency)
        self._pose3 = (ned, (roll, pitch, yaw), latency)

        cov = [0.0] * 21
        cov[COV_XX] = cov[COV_YY] = cov[COV_ZZ] = POS_VAR
        cov[COV_RR] = cov[COV_PP] = cov[COV_YAW] = ANG_VAR

        if self.mode == 2:
            if self.dyaw is None:
                self.n_no_dyaw += 1
                if self.n_no_dyaw % 100 == 1:
                    rospy.logwarn('[vision] mode=2 但 δ 还没测出来（需要静止 + 飞控 '
                                  'ATTITUDE 配对）→ 本轮不发')
            else:
                try:
                    self.master.mav.send(self._make_vpe(fc_usec, ned_s, rpy_s, cov))
                    self.n_sent += 1
                    self._n_sent_win += 1
                except Exception as e:  # noqa: BLE001
                    rospy.logwarn('[vision] 发送失败: %s', e)
                    return

        elif self.mode >= 3:
            # 331：送【原始视觉系】（ned/q_ned_frd 同系），不补 δ —— EKF 自己用
            # _ev_q_error_filt 转进它的 NED（ev_pos_control.cpp:88 的 FRD 分支无
            # yaw_align 要求）。补了 δ 反而少转一次，见文件头 ★。
            try:
                self.master.mav.send(self._make_odometry(fc_usec, ned, q_ned_frd, cov))
                self.n_sent += 1
                self._n_sent_win += 1
            except Exception as e:  # noqa: BLE001
                rospy.logwarn('[vision] 发送失败: %s', e)
                return

        if self.csv_vision is not None:
            try:
                ex = self.ekf_ned or (float('nan'),) * 6
                self.csv_vision.write(
                    '%.3f,%.6f,%.4f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,'
                    '%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%d,'
                    '%.4f,%.4f,%.4f,0x%x\n'
                    % (now, stamp, latency, self.mode, fc_usec,
                       p.x, p.y, p.z, ned_s[0], ned_s[1], ned_s[2],
                       math.degrees(rpy_s[0]), math.degrees(rpy_s[1]),
                       math.degrees(rpy_s[2]),
                       float('nan') if self.dyaw is None else math.degrees(self.dyaw),
                       ned[0], ned[1], ned[2],
                       math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                       self.odom_reset_n,
                       ex[0], ex[1], ex[2], self.est_flags))
            except Exception:  # noqa: BLE001
                pass

    def _make_odometry(self, fc_usec, ned, q_frd, cov):
        """构造 ODOMETRY(331)，frame_id = child_frame_id = MAV_FRAME_LOCAL_FRD。

        位置和四元数【必须同系】：这里送 VINS 世界系（NED 轴约定、未补 δ）的原值，
        EKF 用 _ev_q_error_filt 自己把它转进 EKF 的 NED（ev_pos_control.cpp:88）。
        速度一律 NaN ⇒ PX4 那边 velocity_frame 保持 UNKNOWN、vel 保持 NaN ⇒ 不融合
        速度（本机 EKF2_EV_CTRL=3 也没开速度位）。reset_counter 只在视觉世界系
        真跳变时 +1，用来让 EKF 重置水平位置而不是把跳变当量测。
        """
        cls = mavutil.mavlink.MAVLink_odometry_message
        # EKF2 要求 |q|=1±1e-5 且各分量 ≤1（EKF2.cpp:2311 orientation_valid），
        # 否则姿态被静默丢弃 —— 那样 R_ev_to_ekf 就学不出来了。
        n = math.sqrt(sum(c * c for c in q_frd))
        # ★ 换序：(x,y,z,w) → MAVLink 规定的 [w,x,y,z]。不换 = 飞控读到乱序四元数，
        # R_ev_to_ekf 学出错旋转，EKF 位置被整体转错（2026-10-09 四元数序 bug）。
        qn = ([q_frd[3] / n, q_frd[0] / n, q_frd[1] / n, q_frd[2] / n]
              if n > 1e-9 else [1.0, 0.0, 0.0, 0.0])
        nan = float('nan')
        return cls(time_usec=fc_usec,
                   frame_id=MAV_FRAME_LOCAL_FRD,
                   child_frame_id=MAV_FRAME_LOCAL_FRD,
                   x=ned[0], y=ned[1], z=ned[2],
                   q=qn,
                   vx=nan, vy=nan, vz=nan,
                   rollspeed=nan, pitchspeed=nan, yawspeed=nan,
                   pose_covariance=cov,
                   velocity_covariance=[0.0] * 21,
                   reset_counter=self.odom_reset_n,
                   estimator_type=MAV_ESTIMATOR_TYPE_VIO,
                   quality=100)

    def _make_vpe(self, fc_usec, ned, rpy, cov):
        """构造 VISION_POSITION_ESTIMATE(102)。

        优先带 covariance（v2 的扩展字段，PX4 靠它决定要不要信任这帧）。
        万一 dialect 退回 7 字段的老版本，就只发基础字段并已在启动时告警。
        """
        cls = mavutil.mavlink.MAVLink_vision_position_estimate_message
        base = (fc_usec, ned[0], ned[1], ned[2], rpy[0], rpy[1], rpy[2])
        if self.vpe_fields >= 9:
            return cls(*(base + (cov, 0)))          # + covariance + reset_counter
        if self.vpe_fields >= 8:
            return cls(*(base + (cov,)))            # + covariance
        return cls(*base)                           # 老 dialect：无协方差（已告警）

    # ------------------------------------------------------------ 1Hz 状态行
    def _status_line(self, now):
        if now - self._t_status < 1.0:
            return
        self._t_status = now
        # 速率用"上一秒窗口"算（原来除以 node 启动至今的总时长，一直显示 0.x Hz）
        rate = self._n_sent_win / max(1e-9, now - self._t_win)
        self._t_win = now
        self._n_sent_win = 0
        if self._pose is None:
            rospy.loginfo('[vision] mode=%d 还没收到 odom（等 VINS）', self.mode)
            return
        # ★ 延迟要显示【当前这一帧】的（last_latency），不是最后发出去那帧的。
        # 曾经显示的是 _pose 里的旧值：停发几分钟了还印"延迟=492ms"，看着像正常。
        latency = self.last_latency
        stop = now - self._t_fresh
        stop_s = '' if stop < 2.0 else ' ⚠已停发%.0fs' % stop
        # mode3 发的是原始视觉系，Δ 不要求小（两边原点/朝向本来就不同），
        # 看的是【跟不跟随】；mode2 发的才是真北 NED，Δ 才有"该接近"的含义。
        if self.mode >= 3 and self._pose3 is not None:
            ned, rpy_s, _ = self._pose3
            src = 'FRD原始'
        else:
            ned, rpy_s, _ = self._pose
            src = 'NED'
        ex = self.ekf_ned or (float('nan'),) * 6
        d = math.sqrt(sum((ned[i] - ex[i]) ** 2 for i in range(3)))
        names = [nm for b, nm in EST_FLAGS if self.est_flags & (1 << b)]
        dyaw_s = '—' if self.dyaw is None else '%.1f°' % math.degrees(self.dyaw)
        # 倾角差：我们发的（已补 δ 的）VINS 姿态 vs 飞控姿态。
        # VINS 静止时倾角会漂（实测 ~0.1°/s），差大了就是它在漂，不是变换错。
        tilt = float('nan')
        if self.att_rpy is not None:
            tilt = math.degrees(math.hypot(wrap_pi(rpy_s[0] - self.att_rpy[0]),
                                           wrap_pi(rpy_s[1] - self.att_rpy[1])))
        arm = {True: '★已解锁', False: '已上锁'}.get(self.armed, '?')
        rospy.loginfo(
            '[vision] mode=%d(%s) 发=%d(%.1fHz,%ds) 延迟=%.0fms%s δ=%s 倾角差=%.1f° | '
            '最后发/%s (%.3f,%.3f,%.3f) | EKF NED (%.3f,%.3f,%.3f) Δ=%.3fm | '
            'rpy(%.1f,%.1f,%.1f)° | flags=0x%x[%s] %s',
            self.mode, MODE_DESC.get(self.mode, '?'), self.n_sent, rate,
            int(now - self.t_first),
            latency * 1e3, stop_s, dyaw_s, tilt, src,
            ned[0], ned[1], ned[2], ex[0], ex[1], ex[2], d,
            math.degrees(rpy_s[0]), math.degrees(rpy_s[1]), math.degrees(rpy_s[2]),
            self.est_flags, ','.join(names), arm)
