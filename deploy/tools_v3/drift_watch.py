#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""漂移归因看门狗：位置一飘，就当场摊开"相机链 / 视觉链 / 运动链"三条证据，直接判是谁的锅。

要回答的问题（用户原话）：静止时 VIO 漂了，到底是
    (a) 相机线端口被占（图断了 / seq 不涨 / EBUSY 刷屏）
    (b) 暗淡环境特征点太少（视觉饿死）
    (c) 两条输入都正常，是估计器自己的问题
这三种表现都是"位置在飘"，肉眼分不出来 —— 所以必须把判据同时记下来。

采样节奏：每 2 秒一行 CSV；只有判据触发时才打事件行（安静时刷屏会把证据淹掉）。
    相机链：infra1/2 帧率、header.seq 增量、相机进程在不在、camera.log 的 EBUSY/EAGAIN/MIPI
    视觉链：Shi-Tomasi 角点数 + 亮度（参数与 feat_watch.py 完全一致，数字可直接和基线比）
    估计器链：|p| |v| yaw + /vins_estimator/point_cloud 的窗口地标数
    运动链：/imu 陀螺模长 —— 用来把"人在动飞机"和"真漂移"分开（这是最容易误判的一条）

阈值全部沿用仓库里已有的数字，不另发明：
    特征 80   = start_vins_pi.sh 3.5 步的闸门（<80 判视觉饿死）；健康基线 83~108
    EBUSY=0   = 常态；EBUSY 开始涨 = wedge（recover_camera2.sh 的活）
    陀螺      = 静止基线 ~0.003 rad/s；>0.05 rad/s 判为"人在动"，此时位置变化不算漂移
    |v|       = 静止基线 ~5 mm/s；>30 mm/s 且陀螺安静、连续 3 个周期 = 漂移
    位置台阶  = 相邻采样 Δ|p|>20mm 且陀螺安静 —— 这是本栈静止时最常见的"飘"的形态

★ 本脚本只读不写（纯订阅 + 读日志），绝不动任何进程。恢复是 recover_camera2.sh 的事：
  看门狗一旦自动动手，出事的现场就被自己毁掉了。

用法（容器内）：
    source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
    python3 -u drift_watch.py [间隔秒=2] [总时长秒=0 一直跑]
  输出：stdout（事件+每周期一行）；CSV /tmp/vins_pi/drift_watch.csv
"""
import math
import os
import re
import sys
import time
from collections import deque

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud

try:
    import cv2
    import numpy as np
except Exception:  # noqa: BLE001
    cv2 = None

# ---------------- 判据阈值（来源于仓库既有基线，别随手调） ----------------
FEAT_MIN = 80          # start_vins_pi.sh 的特征闸门
FEAT_GOOD = 100        # 健康线
SPEED_TH = 0.030       # m/s，静止基线 ~0.005
GYRO_MOTION = 0.050    # rad/s，超过就是人在动
STEP_M = 0.020         # 单采样位置台阶
LIFT_M = 0.100         # 120s 窗口内位置抬升
IMG_HZ_MIN = 20.0      # 相机帧率下限（正常 30）
# /imu 频率下限。★ 2026-10-06 夜修正：这套栈的常态是 ~150 Hz，不是 197。
#   依据 = 桥自己的自检报告（/root/.ros/log/px4_imu.log 里「均速 149.8Hz」），
#   Sep 28 与 Oct 6 两轮逐条一致；桥注释也写明 SET_MESSAGE_INTERVAL 只把
#   HIGHRES_IMU 从 133.3Hz 提到 152.0Hz —— 150 就是这套飞控的常态，不是故障。
IMU_HZ_MIN = 120.0
EAGAIN_WARN = 0.5      # 条/秒（按新增量算，不算累计），健康 ~0.007，wedge 前兆会明显抬升
GOOD_FRAMES = 2        # 判"没饿死"要连续几个周期特征都够
WARMUP_S = 20          # 启动后多久内不判台阶/漂移（VINS 初始化会先吐几拍离谱值）
SANE_P_M = 5.0         # |p| 超过这个数就认为估计器还没收敛（桌面场景物理上界）
SANE_V = 1.0           # m/s，同上
# ★ 2026-10-06 夜那轮"静止飞到 32 万米"的元凶就是下面这一条：
#   初始化时 solveGyroscopeBias() 用初始化窗口里的视觉旋转解陀螺零偏，视觉一薄就解歪
#   （那晚解出 0.0116 rad/s，真值只有 0.00008），之后每个陀螺样本都被减掉一个不存在的
#   零偏 ⇒ 姿态匀速转了 80°/120s ⇒ 重力漏进水平轴 ⇒ 12~15 m/s² 起飞。
#   所以：零偏标定值一旦和实测静止零偏差超过 BIAS_ERR_MAX，就是发散的前兆，先报它。
BIAS_ERR_MAX = 0.005   # rad/s（@120s ≈ 34° 姿态误差）
GYRO_WIN_S = 20.0      # 实测零偏取近多少秒 /imu 的均值
GYRO_STD_MAX = 0.010   # rad/s，窗口标准差超过它 = 这段不是静止，零偏判定跳过

CSV_PATH = os.environ.get("DW_CSV", "/tmp/vins_pi/drift_watch.csv")
CAM_LOG = os.environ.get("DW_CAMLOG", "/tmp/vins_pi/camera.log")
VINS_LOG = os.environ.get("DW_VINSLOG", "/tmp/vins_pi/vins.log")
USB_GLOB_DIR = "/sys/bus/usb/devices"

PATTERNS = {
    "ebusy": "number: 16",           # Device or resource busy
    "eagain": "number: 11",          # Resource temporarily unavailable
    "cam_up": "RealSense Node Is Up",
    "wedge": "watchdog triggered",
    "mipi": "MIPI",
    "asic": "Asic Temperature",
}

# ---------------- 工具 ----------------
def to_mono8(m):
    """infra*_rect_raw 是 mono8，直接 reshape。绕开 cv_bridge（镜像里 Python 绑定是坏的）。"""
    if m.encoding not in ("mono8", "8UC1"):
        return None
    return np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]


def count_features(img):
    if cv2 is None or img is None:
        return None
    pts = cv2.goodFeaturesToTrack(img, maxCorners=150, qualityLevel=0.01,
                                  minDistance=30, blockSize=3)
    return 0 if pts is None else len(pts)


def proc_alive(needle):
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % pid, "rb") as fh:
                if needle.encode() in fh.read():
                    return True
        except OSError:
            continue
    return False


def rs_dev_dir():
    """找 D430（8086:0ad4）这次开机挂在哪个 bus 上。别写死 2-1：
    插到别的口、或板子重启后，巴士号会变（2026-10-06 晚上就从 2-1 变成 5-1）。"""
    try:
        names = sorted(os.listdir(USB_GLOB_DIR))
    except OSError:
        return None
    for name in names:
        d = os.path.join(USB_GLOB_DIR, name)
        try:
            v = open(os.path.join(d, "idVendor")).read().strip()
            p = open(os.path.join(d, "idProduct")).read().strip()
        except OSError:
            continue
        if v == "8086" and p == "0ad4":
            return name
    return None


def rs_port_tag():
    """相机的挂牌字符串：'5-1@480M'。速度 480=USB2 口（VINS 双 IR 可能带宽不够），5000=USB3。"""
    dev = rs_dev_dir()
    if not dev:
        return "相机不在总线上"
    try:
        sp = open(os.path.join(USB_GLOB_DIR, dev, "speed")).read().strip()
    except OSError:
        sp = "?"
    return "%s@%sM" % (dev, sp)


def usb_drivers():
    """接口绑在谁身上：usbfs=正常（librealsense 直接用），uvcvideo=wedge 态。"""
    dev = rs_dev_dir()
    if not dev:
        return ["相机不在总线上"]
    out = []
    try:
        for name in sorted(os.listdir(USB_GLOB_DIR)):
            if not name.startswith(dev + ":1."):
                continue
            p = os.path.join(USB_GLOB_DIR, name, "driver")
            try:
                out.append(name[len(dev) + 1:] + "=" + os.path.basename(os.path.realpath(p)))
            except OSError:
                out.append(name[len(dev) + 1:] + "=none")
    except OSError:
        return ["n/a"]
    return out or ["n/a"]


class LogTail(object):
    """增量读 camera.log，累计关键字计数。处理 roslaunch 换文件（inode 变化）与截断。"""

    def __init__(self, path):
        self.path = path
        self.fh = None
        self.ino = None
        self.size = 0
        self.buf = ""
        self.counts = dict((k, 0) for k in PATTERNS)
        self.missing = False

    def poll(self):
        try:
            st = os.stat(self.path)
        except OSError:
            self.missing = True
            return
        self.missing = False
        if self.fh is None or st.st_ino != self.ino or st.st_size < self.size:
            if self.fh:
                self.fh.close()
            try:
                self.fh = open(self.path, "rb")
            except OSError:
                self.fh = None
                return
            self.ino = st.st_ino
            self.size = 0
            self.buf = ""
        data = self.fh.read()
        self.size += len(data)
        if not data:
            return
        self.buf += data.decode("utf-8", "ignore")
        *lines, self.buf = self.buf.split("\n")
        for line in lines:
            for key, pat in PATTERNS.items():
                if pat in line:
                    self.counts[key] += 1


def read_last_gyrbias(path, head_bytes=65536, tail_bytes=16384):
    """找 vins.log 里最后一条 'gyroscope bias initial calibration x y z'。
    返回 (id, (x,y,z))；没有则 (None, None)。id 用行首时间戳 —— vins.log 每次启动
    被截断，同一条不会重复告警。

    ★ 必须【头尾都读】：初始化那段永远在文件开头（实测 934KB 的日志里它在第 30 行，
      按 ~2KB/s 的增长速度，只读尾部 16KB 等于只看最后 8 秒，一定漏掉）。尾部再补一份
      是防将来改成追加写。不维护增量状态，永远不会和文件增长错位。"""
    chunks = []
    try:
        sz = os.path.getsize(path)
        with open(path, "rb") as fh:
            chunks.append(fh.read(head_bytes))
            if sz > head_bytes + tail_bytes:
                fh.seek(sz - tail_bytes)
                chunks.append(fh.read(tail_bytes))
    except OSError:
        return None, None
    live = []
    for ch in chunks:
        live += re.findall(
            r"\[(\d+\.\d+)\]: gyroscope bias initial calibration\s+"
            r"(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)",
            ch.decode("utf-8", "ignore"))
    if not live:
        return None, None
    sid, x, y, z = live[-1]
    return sid, (float(x), float(y), float(z))


def gyro_mean(buf, win_s):
    """近 win_s 秒 /imu 陀螺三分量均值 + 模长标准差。样本 < 50 返回 None。
    静止时均值≈真零偏（2026-10-06 实测 |bias| = 0.00008 rad/s，几乎为零）。"""
    if not buf:
        return None
    t_end = buf[-1][0]
    xs = [r for r in buf if t_end - r[0] <= win_s]
    n = len(xs)
    if n < 50:
        return None
    mx = sum(r[1] for r in xs) / n
    my = sum(r[2] for r in xs) / n
    mz = sum(r[3] for r in xs) / n
    mags = [math.sqrt(r[1] * r[1] + r[2] * r[2] + r[3] * r[3]) for r in xs]
    mu = sum(mags) / n
    sd = math.sqrt(sum((v - mu) ** 2 for v in mags) / n)
    return (mx, my, mz), sd


# ---------------- 状态 ----------------
class State(object):
    def __init__(self):
        self.seq = {}
        self.hz = {}
        self.n_img = {"infra1": 0, "infra2": 0}
        self.frame = {}
        self.feat = {}
        self.bright = {}
        self.imu_n = 0
        self.imu_hz = 0.0
        self.gyro_buf = deque()      # (t, gx, gy, gz)，给"实测静止零偏"用
        self.gyro = float("nan")
        self.acc = float("nan")
        self.p = None
        self.v = None
        self.rpy = (float("nan"),) * 3
        self.p_imu = None
        self.landmarks = -1


def mk_img_cb(st, key):
    def cb(m):
        st.n_img[key] += 1
        st.seq[key] = m.header.seq
        st.frame[key] = m
    return cb


def mk_odom_cb(st, attr):
    def cb(m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if attr == "vio":
            st.p = (p.x, p.y, p.z)
            st.v = (m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z)
            st.rpy = (roll, pitch, yaw)
        else:
            st.p_imu = (p.x, p.y, p.z)
    return cb


def mk_imu_cb(st):
    def cb(m):
        st.imu_n += 1
        g = m.angular_velocity
        a = m.linear_acceleration
        st.gyro = math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z)
        st.acc = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        st.gyro_buf.append((time.time(), g.x, g.y, g.z))
    return cb


def mk_cloud_cb(st):
    def cb(m):
        st.landmarks = len(m.points)
    return cb


def norm3(t):
    return float("nan") if t is None else math.sqrt(sum(c * c for c in t))


def main():
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    rospy.init_node("drift_watch", anonymous=True, disable_signals=False)
    st = State()

    rospy.Subscriber("/camera/infra1/image_rect_raw", Image, mk_img_cb(st, "infra1"), queue_size=1)
    rospy.Subscriber("/camera/infra2/image_rect_raw", Image, mk_img_cb(st, "infra2"), queue_size=1)
    rospy.Subscriber("/vins_estimator/odometry", Odometry, mk_odom_cb(st, "vio"), queue_size=20)
    rospy.Subscriber("/vins_estimator/imu_propagate", Odometry, mk_odom_cb(st, "prop"), queue_size=20)
    rospy.Subscriber("/imu", Imu, mk_imu_cb(st), queue_size=500)
    rospy.Subscriber("/vins_estimator/point_cloud", PointCloud, mk_cloud_cb(st), queue_size=5)

    tail = LogTail(CAM_LOG)
    tail.poll()   # 先把已有日志读一遍，启动横幅里就能给出基线计数

    print("=" * 100)
    print("  drift_watch 启动  %s   间隔 %.1fs  CSV=%s" % (time.strftime("%F %T"), interval, CSV_PATH))
    print("  判据: 特征<%d=饿死; EBUSY涨=端口被占; 陀螺>%.3f=人在动(不算漂移); |v|>%.0fcm/s 且陀螺安静=漂移"
          % (FEAT_MIN, GYRO_MOTION, SPEED_TH * 100))
    print("  基线: 相机口 %s  驱动 %s  相机进程 %s  camera.log 已有 EBUSY=%d EAGAIN=%d MIPI=%d AsicTemp=%d 上线=%d"
          % (rs_port_tag(), "/".join(usb_drivers()), u"✓" if proc_alive("RealSenseNodeFactory") else u"✗不在",
             tail.counts["ebusy"], tail.counts["eagain"], tail.counts["mipi"],
             tail.counts["asic"], tail.counts["cam_up"]))
    print("=" * 100)

    csv = open(CSV_PATH, "a")
    if os.path.getsize(CSV_PATH) == 0:
        csv.write("t_wall,t_ros,p_norm,v_norm,roll,pitch,yaw,p_imu,gyro,acc,imu_hz,"
                  "hz1,hz2,seq1,seq2,feat1,feat2,b1,b2,landmarks,"
                  "ebusy,eagain,cam_up,mipi,asic,cam_proc,vins_proc,relay_proc,driver,camlog\n")
    csv.flush()

    hist = deque()            # (t, p_norm, gyro)
    fast_streak = 0
    good_streak = 0
    drift_active = False
    last_report = 0.0
    first = True
    t_start = time.time()
    prev_p = None
    prev_quiet = False
    prev_seq = {}
    ebusy_prev = 0
    mipi_prev = 0
    asic_prev = 0
    usb_warned = False
    eagain_prev = 0
    eagain_warn_t = 0.0
    warned = {}
    bias_seen = None       # 已对拍过的 VINS 零偏标定行 id

    def warn_once(key, t, msg, every=30.0):
        """同类告警最多每 30 秒一条 —— 持续故障别每周期刷屏。"""
        if t - warned.get(key, -1e9) > every:
            warned[key] = t
            print(msg)

    t_prev = t_start
    while not rospy.is_shutdown():
        t_cycle = time.time()
        if duration and t_cycle - t_start > duration:
            break
        time.sleep(interval)

        # --- 采样窗口 = 上一周期到现在，用【实际】经过时间 ---------------------
        # ★ 别拿名义 interval 当分母：每周期还要找角点、读日志，实测周期比 interval
        #   长 ~10%，用名义值会把帧率/IMU 频率系统性偏高（2026-10-06 夜改）。
        dt_win = max(1e-6, t_cycle - t_prev)
        t_prev = t_cycle
        hz = {}
        for k in ("infra1", "infra2"):
            hz[k] = st.n_img[k] / dt_win
            st.n_img[k] = 0
        imu_hz = st.imu_n / dt_win
        st.imu_n = 0
        while st.gyro_buf and t_cycle - st.gyro_buf[0][0] > GYRO_WIN_S + 5.0:
            st.gyro_buf.popleft()      # 陀螺窗口只留 GYRO_WIN_S+5 秒

        tail.poll()
        c = tail.counts
        drivers = usb_drivers()
        cam_proc = proc_alive("RealSenseNodeFactory")
        vins_proc = proc_alive("vins_node")
        relay_proc = proc_alive("vins_diag_relay")

        # 特征只在有新帧时算
        for k in ("infra1", "infra2"):
            img = st.frame.get(k)
            if img is None:
                st.feat[k], st.bright[k] = None, None
                continue
            try:
                arr = to_mono8(img)
                st.feat[k] = count_features(arr)
                st.bright[k] = float(arr.mean()) if arr is not None else None
            except Exception:  # noqa: BLE001
                st.feat[k], st.bright[k] = None, None

        p_norm = norm3(st.p)
        v_norm = norm3(st.v)
        p_imu = norm3(st.p_imu)
        ts = time.strftime("%H:%M:%S")

        hist.append((t_cycle, p_norm, st.gyro))
        while hist and t_cycle - hist[0][0] > 120:
            hist.popleft()
        p_min = min(h[1] for h in hist) if hist else p_norm

        motion = (st.gyro == st.gyro) and st.gyro > GYRO_MOTION
        quiet = not motion

        # 初始化/未收敛窗口：VINS 刚起来时 odometry 会先吐几拍离谱值
        # （实测 |p|=180m |v|=25m/s），那既不是台阶也不是漂移，别误报。
        # 判据：还没跑过热身时长，或数值超出桌面场景的物理上界。
        warm = (t_cycle - t_start) < WARMUP_S
        naive = warm or p_norm != p_norm or p_norm > SANE_P_M or v_norm != v_norm or v_norm > SANE_V

        # --- 判 1: 位置台阶（静止时最常见的"飘"） ---
        # 只在"上一个周期也是安静"时判，避免运动刚停下的那一个采样被误报成台阶
        if (not naive and prev_p is not None and quiet and prev_quiet
                and abs(p_norm - prev_p) > STEP_M):
            d = p_norm - prev_p
            print(u"▲ [%s] 位置台阶 %+.0fmm  |p| %.4f→%.4f  |v|=%.4f  陀螺 %.4f "
                  u"| 特征 %s/%s 图 %.1f/%.1fHz  EBUSY %d EAGAIN %d"
                  % (ts, d * 1000, prev_p, p_norm, v_norm, st.gyro,
                     st.feat.get("infra1"), st.feat.get("infra2"), hz["infra1"], hz["infra2"],
                     c["ebusy"], c["eagain"]))
        prev_p = p_norm
        prev_quiet = quiet

        # --- 判 2: 漂移（速度持续超基线且陀螺安静） ---
        if naive:
            fast_streak = 0     # 初始化窗口不攒证据
        elif quiet and v_norm > SPEED_TH:
            fast_streak += 1
        else:
            fast_streak = max(0, fast_streak - 1)
        lift = p_norm - p_min
        is_drift = (not naive) and quiet and (fast_streak >= 3 or lift > LIFT_M)

        if is_drift and (not drift_active or t_cycle - last_report > 30):
            drift_active = True
            last_report = t_cycle
            feats_ok = all((st.feat.get(k) or 0) >= FEAT_MIN for k in ("infra1", "infra2"))
            cam_ok = (hz["infra1"] >= IMG_HZ_MIN and hz["infra2"] >= IMG_HZ_MIN
                      and cam_proc and c["ebusy"] == 0)
            cam_stall = (hz["infra1"] < IMG_HZ_MIN or hz["infra2"] < IMG_HZ_MIN
                         or not cam_proc or c["ebusy"] > 0)
            print(u"")
            print(u"★★★ [%s] 漂移事件  |p|=%.3f m（120s 内抬升 %.3f）  |v|=%.4f m/s  陀螺 %.4f rad/s（安静）"
                  % (ts, p_norm, lift, v_norm, st.gyro))
            print(u"    相机: %.1f/%.1f Hz  seq %s/%s  进程%s  EBUSY %d  EAGAIN %d  驱动 %s"
                  % (hz["infra1"], hz["infra2"], st.seq.get("infra1"), st.seq.get("infra2"),
                     u"✓" if cam_proc else u"✗不在", c["ebusy"], c["eagain"], "/".join(drivers)))
            print(u"    视觉: 特征 %s/%s（闸门 %d）  亮度 %s/%s  窗口地标 %d"
                  % (st.feat.get("infra1"), st.feat.get("infra2"), FEAT_MIN,
                     st.bright.get("infra1"), st.bright.get("infra2"), st.landmarks))
            if cam_stall:
                print(u"    ⇒ 判定: 【相机链断了】图像不流/端口被占 —— 先跑 recover_camera2.sh，不是特征的锅")
            elif not feats_ok:
                print(u"    ⇒ 判定: 【视觉饿死】图在流但特征 < %d（太暗/无纹理/对着白墙）—— 开灯或转视角"
                      % FEAT_MIN)
            else:
                print(u"    ⇒ 判定: 【两条输入都正常】相机在流、特征够 —— 不是端口占据、不是暗环境，"
                      u"是估计器内部（看台阶是否伴随地标/特征突变）")
            print(u"")
        elif drift_active and not is_drift:
            print(u"── [%s] 漂移事件结束  |p|=%.4f  陀螺 %.4f" % (ts, p_norm, st.gyro))
            drift_active = False

        # --- 判 3: 饿死（连续 GOOD_FRAMES 个周期特征不足） ---
        if all((st.feat.get(k) or 999) < FEAT_MIN for k in ("infra1", "infra2")):
            good_streak += 1
            if good_streak == GOOD_FRAMES:
                print(u"✗ [%s] 视觉饿死: 特征 %s/%s < %d  亮度 %s/%s  图 %.1f/%.1fHz"
                      % (ts, st.feat.get("infra1"), st.feat.get("infra2"), FEAT_MIN,
                         st.bright.get("infra1"), st.bright.get("infra2"), hz["infra1"], hz["infra2"]))
        else:
            if good_streak >= GOOD_FRAMES:
                print(u"✓ [%s] 视觉恢复: 特征 %s/%s" % (ts, st.feat.get("infra1"), st.feat.get("infra2")))
            good_streak = 0

        # --- 判 4: 相机链异常（断流 / wedge / 进程死 / EAGAIN 抬头） ---
        for k in ("infra1", "infra2"):
            if st.seq.get(k) is not None and hz[k] < IMG_HZ_MIN and st.seq.get(k) == prev_seq.get(k):
                warn_once("stall_" + k, t_cycle,
                          u"✗ [%s] 相机断流: %s %.1fHz seq 停在 %s" % (ts, k, hz[k], st.seq.get(k)))
            prev_seq[k] = st.seq.get(k)
        if not cam_proc:
            warn_once("camproc", t_cycle,
                      u"✗ [%s] 相机进程不在了（nodelet 静默退出形态：EBUSY=0 也照样是死的）" % ts)
        if c["ebusy"] > 0 and c["ebusy"] != ebusy_prev:
            print(u"✗ [%s] EBUSY 计数 %d（wedge 形态）驱动=%s ⇒ recover_camera2.sh"
                  % (ts, c["ebusy"], "/".join(drivers)))
        ebusy_prev = c["ebusy"]
        if c["eagain"] > eagain_prev:
            rate = (c["eagain"] - eagain_prev) / max(interval, 1e-9)
            if rate > EAGAIN_WARN and t_cycle - eagain_warn_t > 30:
                eagain_warn_t = t_cycle
                print(u"⚠ [%s] EAGAIN 新增 %.1f 条/秒（wedge 前兆，健康 ~0.007）"
                      % (ts, rate))
        eagain_prev = c["eagain"]
        if (c["mipi"], c["asic"]) != (mipi_prev, asic_prev):
            print(u"⚠ [%s] 物理层指纹: MIPI=%d AsicTemp=%d（见 camera-zero-frames-hw-fault）"
                  % (ts, c["mipi"], c["asic"]))
        mipi_prev, asic_prev = c["mipi"], c["asic"]
        port = rs_port_tag()
        if not usb_warned and "@5000M" not in port:
            usb_warned = True
            print(u"⚠ [%s] 相机口 %s —— 不是 USB3(5000M)！双 IR 30Hz 可能带宽不够，"
                  u"会表现为低帧率/丢帧（2026-10-06 晚从 2-1@5000M 变成过 5-1@480M）" % (ts, port))
        if imu_hz < IMU_HZ_MIN:
            warn_once("imu", t_cycle,
                      u"✗ [%s] /imu 只有 %.1f Hz（正常 ~150，<%.0f 才算桥/串口出问题）"
                      % (ts, imu_hz, IMU_HZ_MIN))

        # --- 判 5: VINS 初始化零偏标定 vs 实测静止零偏（2026-10-06 夜那次的元凶） ---
        # 桥解歪了零偏 ⇒ 姿态匀速转 ⇒ 重力漏进水平轴 ⇒ 发散。视觉薄的时候必查这里，
        # 别去怀疑相机线/端口 —— 那晚相机链全程健康（30.5Hz、usbfs、EBUSY=0）。
        try:
            bid, bv = read_last_gyrbias(VINS_LOG)
            if bid is not None and bid != bias_seen:
                bias_seen = bid
                bmag = math.sqrt(bv[0] * bv[0] + bv[1] * bv[1] + bv[2] * bv[2])
                print(u"◆ [%s] VINS 零偏标定 %+.5f %+.5f %+.5f rad/s（|%.5f|）"
                      % (ts, bv[0], bv[1], bv[2], bmag))
                gm = gyro_mean(st.gyro_buf, GYRO_WIN_S)
                if gm is None:
                    print(u"    实测零偏: /imu 窗口样本不够，跳过对拍")
                else:
                    (mx, my, mz), sd = gm
                    err = math.sqrt((bv[0] - mx) ** 2 + (bv[1] - my) ** 2
                                    + (bv[2] - mz) ** 2)
                    if sd > GYRO_STD_MAX:
                        print(u"    实测零偏 %+.5f %+.5f %+.5f（标准差 %.4f > %.3f："
                              u"这段不是静止，不判定）" % (mx, my, mz, sd, GYRO_STD_MAX))
                    else:
                        print(u"    实测零偏 %+.5f %+.5f %+.5f（标准差 %.4f）  差 %.5f rad/s"
                              % (mx, my, mz, sd, err))
                        if err > BIAS_ERR_MAX:
                            print(u"    ✗✗ 零偏标定失配！姿态会以 %.5f rad/s 匀速转"
                                  u"（≈%.0f°/120s）；视觉拉不回来就会重力漏进水平轴、直接发散。"
                                  u"本次若飘，先看这行，不是相机线/端口的问题。"
                                  % (err, math.degrees(err) * 120))
        except Exception as e:  # noqa: BLE001
            print(u"    （零偏对拍出错，忽略：%s）" % e)

        # --- CSV 一行 ---
        row = [time.time(), rospy.get_time(), p_norm, v_norm,
               st.rpy[0], st.rpy[1], st.rpy[2], p_imu, st.gyro, st.acc, imu_hz,
               hz["infra1"], hz["infra2"], st.seq.get("infra1"), st.seq.get("infra2"),
               st.feat.get("infra1"), st.feat.get("infra2"), st.bright.get("infra1"),
               st.bright.get("infra2"), st.landmarks,
               c["ebusy"], c["eagain"], c["cam_up"], c["mipi"], c["asic"],
               int(cam_proc), int(vins_proc), int(relay_proc),
               "%s:%s" % (port, "/".join(drivers)), tail.size]
        csv.write(",".join("" if x is None else ("%.6f" % x if isinstance(x, float) else str(x))
                           for x in row) + "\n")
        csv.flush()

        # --- 每周期一行（心跳；特征用 -- 表示还没测到） ---
        if first or int(t_cycle - t_start) % 30 < interval:
            print(u"  [%s] |p| %s |v| %s yaw %s | 图 %.1f/%.1fHz IMU %.0fHz | 特征 %s/%s 亮 %s/%s 地标 %s | "
                  u"陀螺 %.4f EBUSY %d EAGAIN %d"
                  % (ts, "%.4f" % p_norm if p_norm == p_norm else "--",
                     "%.4f" % v_norm if v_norm == v_norm else "--",
                     "%.1f" % st.rpy[2] if st.rpy[2] == st.rpy[2] else "--",
                     hz["infra1"], hz["infra2"], imu_hz,
                     st.feat.get("infra1"), st.feat.get("infra2"),
                     st.bright.get("infra1"), st.bright.get("infra2"), st.landmarks,
                     st.gyro, c["ebusy"], c["eagain"]))
        first = False

    csv.close()
    print(u"  drift_watch 退出  %s" % time.strftime("%F %T"))


if __name__ == "__main__":
    main()
