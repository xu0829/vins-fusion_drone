#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""px4_live.py —— 实时对比：飞控镜像遥测 vs 我们发出去的位姿（只读，绝不碰串口）。

为什么需要它：`tail -f /tmp/imu_node_recover.log` 看到的是一行行 1Hz 状态行，
数字挤在一起，看不出"我们发出去的点"和"飞控 EKF 里实际的点"是不是同一个。
本脚本把两侧并排摆出来，并直接给判据。

数据来源（全部是 px4_imu_node 已经镜像出来的 ROS 话题，本脚本只订阅）：
    /px4/local_position_ned   PoseStamped  飞控 EKF 位置(NED) + 飞控姿态(四元数)
    /px4/estimator_status     String       flags/ratio —— 判断"融合上没上"
    /px4/statustext           String       飞控控制台（ECL_INFO/WARN）最近一条
    /vins_estimator/odometry  Odometry     VINS 原始输出（我们发的东西的源）
（飞控速度 vN/vE/vD 只在 /tmp/vins_pi/px4_telemetry.csv 里，没镜像成话题；
 要看速度用 `tail -f` 那个 CSV，或给镜像加话题——都属改动，本脚本不做。）

对比逻辑（mode2/mode3 落点是同一个，所以判据也同一个）：
    mode3 发【原始视觉系】ned        → PX4 自己用 R_ev_to_ekf(≈Rz(δ)) 转进 NED
    mode2 发【已补 δ】Rz(δ)·ned      → PX4 当 NED 直接用
    两条路的预期都是   EKF ≈ Rz(δ)·ned   ⇒ 水平 Δ 应为厘米级。
★ Δ 随 |p| 成比例放大 = "整体转过一个角"的指纹（四元数序/坐标系错位一类），
  不是漂移。2026-10-09 的四元数序 bug 就是靠这个指纹认出来的。
★ |p| 很小（原点附近）时两种模型都重合，Δ 小【不能】证明正确 —— 当年
  "RMS 0.2cm" 的假阳性就是这么来的。所以 |p|<0.15m 时本脚本拒绝下结论。

安全：只订阅话题 + 只读模式文件；不开串口、不发任何 MAVLink。
      与 vision_uplink 同目录，坐标变换直接复用它的函数（单一约定来源，
      避免本脚本自己抄一份再抄错——bug 就是这么来的）。

用法（容器内）：
    python3 /root/xu_drone/live/px4_live.py              # 实时，1Hz 刷屏
    python3 /root/xu_drone/live/px4_live.py --hz 2
    python3 /root/xu_drone/live/px4_live.py --line       # ★ 一秒一行，像 tail -f，能直接存文件
    python3 /root/xu_drone/live/px4_live.py --plain      # 每秒一整块顺序打印（给日志/重定向）
    python3 /root/xu_drone/live/px4_live.py --line --duration 600 > /tmp/watch.txt
    python3 /root/xu_drone/live/px4_live.py --selftest   # 数值自检后退出（需在容器内跑）

--line 一行里都有什么（从左到右）：
    时刻 | 视觉 ned(N,E,D) |p|  →  预期 EKF(N,E)  EKF 实际(N,E,D)  →  水平Δ 垂直Δ  δ
    挪飞机时盯着 视觉/预期/EKF 三对数字一起动、Δh 保持 mm 级 = 正常。
"""
import argparse
import collections
import math
import os
import re
import sys
import time

MODE_FILE = os.environ.get('VISION_MODE_FILE', '/root/xu_drone/live/vision_uplink.mode')

# 判据阈值
HORIZ_OK_M = 0.05          # 水平 Δ 到厘米级才算"同点"
VERT_OK_M = 0.30           # 垂直：高度基准差（气压/测距/原点）允许大一些
MIN_ABS_P_M = 0.15         # |p| 小于这个数，Δ 判据不可信（原点附近两模型重合）
STALE_S = 2.0              # 话题超过这么久没更新 = 没在发

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)      # 保证能 import 到同目录的 vision_uplink


# ------------------------------------------------------------------ 小工具
def wrap_pi(x):
    """角度规整到 (−π, π]。"""
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def rz_xy(x, y, ang_rad):
    """绕竖直轴转 ang：视觉系 → 飞控真北 NED（与 vision_uplink 的补 δ 同式）。"""
    cd, sd = math.cos(ang_rad), math.sin(ang_rad)
    return (cd * x - sd * y, sd * x + cd * y)


def _f(v, w=7, p=3):
    return '%*.*f' % (w, p, v) if math.isfinite(v) else ' ' * (w - 1) + '—'


def _read_mode():
    try:
        with open(MODE_FILE) as f:
            return int(f.read().strip().split()[0])
    except Exception:  # noqa: BLE001
        return -1


def vins_to_visual(position, orientation):
    """odom 的 (position, orientation) → (ned, rpy)，与 vision_uplink 逐字同式。

    返回 ned=(y,x,-z)：位置在视觉系里、但轴标签按 NED 约定（这就是 mode3 原样发出去的）；
    q_ned_frd 的 rpy 是【原始视觉系】姿态（yaw 与飞控 yaw 差一个常值 δ）。
    """
    import vision_uplink as V          # 同目录；容器内才有 rospy/pymavlink
    ned = (position.y, position.x, -position.z)
    q = orientation
    q_ned_frd = V.qmul(V.qmul(V.Q_ENU2NED, (q.x, q.y, q.z, q.w)), V.Q_FLU2FRD)
    return ned, V.q_to_rpy(q_ned_frd)


def verdict_horiz(dh, p_horiz):
    """给水平 Δ 判词。返回 (符号, 说明)。"""
    if p_horiz < MIN_ABS_P_M:
        return '·', '|p|=%.2fm 太小，两种模型都重合，Δ 判据不可信（挪远点再看）' % p_horiz
    if dh <= HORIZ_OK_M:
        return '✓', '视觉与 EKF 同点（厘米级）'
    ratio = dh / p_horiz
    if ratio > 0.3:
        return '✗', '误差随距离成比例（Δ/|p|=%.2f ≈ 2·sin(Δψ/2)）→ 旋转错位指纹，查四元数序/坐标系' % ratio
    return '✗', '偏差大但不随距离成比例 → 另有原因（漂移/停发/原点未对齐？）'


# ------------------------------------------------------------------ 自检
def _selftest():
    """数值自检：用【独立实现】的显式 DCM 算出期望，再和 vision_uplink 的四元数
    路径对比。刻意不走同一条代码路径 —— 上次四元数序 bug 就是因为自检两边同错
    而"通过"的。真实数据回归点取自 2026-10-09 的 CSV（当时 EKF 与期望差 1.33m，
    这一条锁住的是"变换本身"，不是"当前是否同点"）。"""
    try:
        import vision_uplink as V
    except ImportError as e:
        print('自检需要容器环境（vision_uplink 依赖 rospy/pymavlink）: %s' % e)
        return 3

    n_ok = n_bad = 0

    def chk(name, got, exp, tol):
        nonlocal n_ok, n_bad
        good = all(abs(g - e) <= tol for g, e in zip(got, exp))
        print('  [%s] %s' % ('OK' if good else 'FAIL', name))
        if not good:
            print('        got=%s' % (['%.6f' % v for v in got],))
            print('        exp=%s' % (['%.6f' % v for v in exp],))
        n_ok, n_bad = n_ok + good, n_bad + (not good)

    # --- 独立实现（显式 3x3 矩阵，不碰 vision_uplink） ---
    def dcm(x, y, z, w):
        n = math.sqrt(x * x + y * y + z * z + w * w)
        x, y, z, w = x / n, y / n, z / n, w / n
        return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]

    def mm(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def rpy_of(R):
        return (math.degrees(math.atan2(R[2][1], R[2][2])),
                math.degrees(math.asin(max(-1.0, min(1.0, -R[2][0])))),
                math.degrees(math.atan2(R[1][0], R[0][0])))

    def qmul_ref(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)

    def qz(d):
        h = math.radians(d) / 2.0
        return (0.0, 0.0, math.sin(h), math.cos(h))

    def qy(d):
        h = math.radians(d) / 2.0
        return (0.0, math.sin(h), 0.0, math.cos(h))

    R_enu2ned = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    R_flu2frd = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]

    def expect_rpy(q_world_body_xyzw):
        """ENU/FLU 世界姿态 → 期望的 NED/FRD rpy（度），全程显式矩阵。"""
        R = mm(mm(R_enu2ned, dcm(*q_world_body_xyzw)), R_flu2frd)
        return rpy_of(R)

    def via_vision_uplink(q_world_body_xyzw):
        """同一条变换，但走 vision_uplink 的四元数路径（被测对象）。"""
        q = V.qmul(V.qmul(V.Q_ENU2NED, q_world_body_xyzw), V.Q_FLU2FRD)
        return tuple(math.degrees(v) for v in V.q_to_rpy(q))

    print('px4_live 自检：')

    # 1) 位置轴映射：ENU→NED 就是 (y, x, -z)
    class _P(object):
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class _O(object):
        def __init__(self, x, y, z, w):
            self.x, self.y, self.z, self.w = x, y, z, w

    for name, p, exp in (('北(ENU y)', (0, 1, 0), (1, 0, 0)),
                         ('东(ENU x)', (1, 0, 0), (0, 1, 0)),
                         ('上', (0, 0, 1), (0, 0, -1))):
        ned, _ = vins_to_visual(_P(*p), _O(0, 0, 0, 1))
        chk('位置轴 %s → ned' % name, ned, exp, 1e-9)

    # 2) 姿态：四元数路径 vs 独立 DCM 路径
    cases = (('机体与世界重合(机头指东) → yaw=+90°', (0, 0, 0, 1), (0, 0, 90)),
             ('ENU yaw=+90(机头指北) → yaw=0°', (0, 0, math.sin(math.radians(45)),
                                                 math.cos(math.radians(45))), (0, 0, 0)),
             ('ENU yaw=30 + FLU pitch=10 → NED rpy', tuple(qmul_ref(qz(30), qy(10))),
              (0, -10, 60)))
    for name, q, _exp in cases:
        chk('姿态 %s' % name, via_vision_uplink(q), expect_rpy(q), 1e-6)

    # 3) 真实数据回归：Rz(δ)·ned（2026-10-09 02:49 一帧）
    got = rz_xy(0.7862, -0.1496, math.radians(-134.91))
    chk('真实帧 Rz(δ)·ned（记录 ekf=-0.6611,-0.4512）', got, (-0.6610, -0.4512), 2e-3)

    # 4) 判词
    sym, _ = verdict_horiz(0.03, 0.81)
    chk('判词：Δ=3cm → ✓', (0 if sym == '✓' else 1,), (0.0,), 0.5)
    sym, _ = verdict_horiz(1.33, 0.81)
    chk('判词：Δ=1.33m → ✗', (0 if sym == '✗' else 1,), (0.0,), 0.5)
    sym, _ = verdict_horiz(0.01, 0.02)     # 原点附近：不许给 ✓
    chk('判词：|p|=2cm → 不给结论', (0 if sym == '·' else 1,), (0.0,), 0.5)

    print('自检：%d OK / %d FAIL' % (n_ok, n_bad))
    return 0 if n_bad == 0 else 1


# ------------------------------------------------------------------ 实时视图
class Px4Live(object):
    def __init__(self, hz, plain, duration, line=False):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        self.rospy = rospy
        self.hz, self.plain, self.duration = hz, plain, duration
        self.line = line
        self.odom = None      # (t, ned, rpy_visual)
        self.fc = None        # (t, x, y, z, rpy_fc)
        self.est = None       # (t, text)
        self.stext = None     # (t, text)
        self.hist = collections.deque(maxlen=90)   # 近 90 个 Δh（≈90s）
        self._last_plain_s = None

        rospy.init_node('px4_live', anonymous=True)
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber('/px4/local_position_ned', PoseStamped, self._on_fc, queue_size=10)
        rospy.Subscriber('/px4/estimator_status', String, self._on_est, queue_size=10)
        rospy.Subscriber('/px4/statustext', String, self._on_stext, queue_size=10)

    def _on_odom(self, m):
        # 走 vins_to_visual —— 自检测的就是这条路，别在这里另抄一份变换
        ned, rpy = vins_to_visual(m.pose.pose.position, m.pose.pose.orientation)
        self.odom = (time.time(), ned, rpy)

    def _on_fc(self, m):
        import vision_uplink as V
        q = m.pose.orientation
        self.fc = (time.time(), m.pose.position.x, m.pose.position.y, m.pose.position.z,
                   V.q_to_rpy((q.x, q.y, q.z, q.w)))

    def _on_est(self, m):
        self.est = (time.time(), m.data)

    def _on_stext(self, m):
        self.stext = (time.time(), m.data)

    # ------------------------------------------------------------ 渲染
    def render(self):
        now = time.time()
        L = []
        age = lambda t: now - t  # noqa: E731
        mode = _read_mode()
        L.append('══ PX4 实时对比 %s ══  只读镜像（不开串口），mode=%s'
                 % (time.strftime('%H:%M:%S'), mode if mode >= 0 else '?'))

        # 数据新鲜度
        L.append('  数据新鲜度   odom %.2fs | 飞控位置 %.2fs | flags %.2fs'
                 % (age(self.odom[0]) if self.odom else float('inf'),
                    age(self.fc[0]) if self.fc else float('inf'),
                    age(self.est[0]) if self.est else float('inf')))
        if self.odom is None and self.fc is None:
            L.append('  等待话题…（若一直空：px4_imu_node 没起 / mode=0 连遥测都不请求）')
            self._show(L)
            return

        # 【飞控原始遥测】
        L.append('【飞控原始遥测】')
        if self.fc is not None and age(self.fc[0]) < STALE_S:
            _, fx, fy, fz, frpy = self.fc
            L.append('  EKF 位置 NED    N=%s  E=%s  D=%s' % (_f(fx), _f(fy), _f(fz)))
            L.append('  飞控姿态        roll=%s pitch=%s yaw=%s  (°)'
                     % (_f(math.degrees(frpy[0]), 6, 2), _f(math.degrees(frpy[1]), 6, 2),
                        _f(math.degrees(frpy[2]), 7, 2)))
        else:
            fx = fy = fz = frpy = None
            L.append('  （飞控位置缺失/过旧 —— 串口断了？）')
        if self.est is not None and age(self.est[0]) < 5.0:
            L.append('  融合状态        %s' % self.est[1])
        else:
            L.append('  融合状态        （无）/px4/estimator_status 过旧')
        if self.stext is not None:
            L.append('  控制台(最近)    %s  (%.0fs 前)' % (self.stext[1][:78], age(self.stext[0])))
        else:
            L.append('  控制台(最近)    —（ECL 的 INFO/WARN 不一定产生 STATUSTEXT）')

        # 【我们发的】
        L.append('【我们发的】')
        if self.odom is None:
            L.append('  视觉 odom       还没来（VINS 没起？）')
            self._show(L)
            return
        to, ned, rpy_v = self.odom
        fresh = age(to) < STALE_S
        yaw_v = rpy_v[2]
        L.append('  视觉位置 ned    N=%s  E=%s  D=%s   |p|=%s m   (%.2fs 前)%s'
                 % (_f(ned[0]), _f(ned[1]), _f(ned[2]),
                    _f(math.sqrt(sum(v * v for v in ned)), 6), age(to),
                    '' if fresh else '  ⚠停发?'))
        L.append('  视觉姿态(原始)  rpy=(%s,%s,%s)°  yaw=%s°'
                 % (_f(math.degrees(rpy_v[0]), 6, 2), _f(math.degrees(rpy_v[1]), 6, 2),
                    _f(math.degrees(yaw_v), 7, 2), _f(math.degrees(yaw_v), 7, 2)))
        if mode == 3:
            L.append('  → 发出去的就是上面这个（mode3：原始视觉系，PX4 自己转）')
        elif mode == 2:
            L.append('  → 发出去的是 Rz(δ)·ned（mode2：已补 δ，PX4 直用）')
        else:
            L.append('  → mode=%s：不是本脚本对比的目标模式（0=关 1=dry-run）' % mode)

        # 【对比】
        L.append('【对比】EKF ≟ Rz(δ)·视觉（δ=FC_yaw − 视觉_yaw）')
        if fx is None or frpy is None:
            L.append('  飞控侧没数据，无法对比')
            self._show(L)
            return
        dyaw = wrap_pi(frpy[2] - yaw_v)
        pred_x, pred_y = rz_xy(ned[0], ned[1], dyaw)
        L.append('  δ = %s°   (FC yaw %s° − 视觉 yaw %s°)'
                 % (_f(math.degrees(dyaw), 7, 2), _f(math.degrees(frpy[2]), 7, 2),
                    _f(math.degrees(yaw_v), 7, 2)))
        L.append('  预期 EKF        N=%s  E=%s      （Rz(δ)·视觉）' % (_f(pred_x), _f(pred_y)))
        L.append('  EKF 实际        N=%s  E=%s' % (_f(fx), _f(fy)))
        dh = math.hypot(fx - pred_x, fy - pred_y)
        dv = fz - ned[2]
        p_h = math.hypot(ned[0], ned[1])
        sym, why = verdict_horiz(dh, p_h)
        L.append('  水平 Δ = %s m %s      垂直 Δ = %s m %s'
                 % (_f(dh, 6), sym, _f(dv, 6),
                    '✓' if abs(dv) <= VERT_OK_M else '✗'))
        L.append('    ↳ %s' % why)
        if fresh:
            self.hist.append(dh)
        if len(self.hist) >= 5:
            h = sorted(self.hist)
            L.append('  近 %ds 水平 Δ：中位 %s  最小 %s  最大 %s m'
                     % (len(h), _f(h[len(h) // 2], 6), _f(h[0], 6), _f(h[-1], 6)))
        self._show(L)

    def render_line(self):
        """单行模式：一秒一行。给"盯着看趋势"和"重定向存文件"用，像 tail -f。"""
        now = time.time()
        if self.odom is None and self.fc is None:
            return '%s  等待话题…（px4_imu_node 没起 / mode=0？）' % time.strftime('%H:%M:%S')
        L = [time.strftime('%H:%M:%S')]
        ned = rpy_v = None
        if self.odom is not None:
            to, ned, rpy_v = self.odom
            L.append('视觉(%s,%s,%s)|p|=%.3f%s' % (
                _f(ned[0], 6), _f(ned[1], 6), _f(ned[2], 6), math.hypot(ned[0], ned[1]),
                '' if now - to < STALE_S else '⚠停发'))
        else:
            L.append('视觉 —（VINS 没起？）')
        if self.fc is not None and now - self.fc[0] < STALE_S:
            _, fx, fy, fz, frpy = self.fc
            L.append('EKF(%s,%s,%s)' % (_f(fx, 6), _f(fy, 6), _f(fz, 6)))
            if ned is not None:
                dyaw = wrap_pi(frpy[2] - rpy_v[2])
                px, py = rz_xy(ned[0], ned[1], dyaw)
                dh = math.hypot(fx - px, fy - py)
                sym, _ = verdict_horiz(dh, math.hypot(ned[0], ned[1]))
                L.append('预期(%s,%s)' % (_f(px, 6), _f(py, 6)))
                L.append('Δh=%.3f%s Δv=%s' % (dh, sym, _f(fz - ned[2], 6)))
                L.append('δ=%s°' % _f(math.degrees(dyaw), 6, 1))
        else:
            L.append('EKF —（飞控侧没数据：串口断了？）')
        if self.est is not None and now - self.est[0] < 5.0:
            m = re.search(r'pos_h_ratio=([-\d.eEnan]+)', self.est[1])
            if m:
                L.append('ratio=%s' % m.group(1)[:5])
        return '  '.join(L)

    def _show(self, lines):
        if self.plain:
            s = int(time.time())
            if s != self._last_plain_s:      # 顺序模式最多每秒一块
                self._last_plain_s = s
                print('\n'.join(lines), flush=True)
        else:
            sys.stdout.write('\033[H\033[J' + '\n'.join(lines) + '\n')
            sys.stdout.flush()

    def run(self):
        rospy = self.rospy
        r = rospy.Rate(self.hz)
        t0 = time.time()
        while not rospy.is_shutdown():
            if self.line:
                print(self.render_line(), flush=True)
            else:
                self.render()
            if self.duration and time.time() - t0 >= self.duration:
                break
            r.sleep()


def main():
    ap = argparse.ArgumentParser(description='PX4 飞控遥测 vs 我们发的位姿 实时对比（只读）')
    ap.add_argument('--hz', type=float, default=1.0, help='刷新率（默认 1）')
    ap.add_argument('--plain', action='store_true', help='每秒一整块顺序打印（日志/重定向用）')
    ap.add_argument('--line', action='store_true', help='一秒一行（像 tail -f，存文件/看趋势用）')
    ap.add_argument('--duration', type=float, default=0.0, help='跑这么多秒后退出（默认一直跑）')
    ap.add_argument('--selftest', action='store_true', help='数值自检后退出')
    a = ap.parse_args()
    if a.selftest:
        sys.exit(_selftest())
    if a.plain:
        print('# px4_live 顺序模式：每秒一块；Ctrl-C 退出', flush=True)
    if a.line:
        print('# 时刻  视觉ned(N,E,D)|p|  EKF(N,E,D)  预期(N,E)  Δh Δv  δ', flush=True)
    Px4Live(a.hz, a.plain, a.duration, a.line).run()


if __name__ == '__main__':
    main()
