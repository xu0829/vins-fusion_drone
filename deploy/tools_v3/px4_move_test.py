#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""px4_move_test.py —— 半自动「平移验证」采集：把飞机挪离视觉原点，全程记录
【我们发的】与【飞控 EKF】，跑完自动分段出结论。（只读镜像，绝不碰串口）

为什么必须挪远：四元数序 / 坐标系一类的错位是「整体转过一个角」，位置误差 ∝ |p|；
在视觉原点附近两种模型重合 —— "Δ 只有几毫米"什么也证明不了
（2026-10-09 的假阳性 RMS 0.2cm 就是这么来的）。所以本脚本把"挪远 + 等收敛 + 分段对比"
固化成一条流程。

流程（默认 ~6.5 分钟；只按提示动手，不用按键；中途 Ctrl-C 也会出结论）：
    0:00-0:30  基准   保持静止（当前 |p| 若已 >0.3m 可跳过）
    0:30-2:00  平移①  沿桌面推 ≥1 米（贴桌面、保持水平、别挡相机），到位松手
    2:00-4:00  保持①  不要碰，等飞控 EKF 重学 q_error（Δ 会自己往下走）
    4:00-5:00  平移②  换个方向（与刚才大致垂直）再推 ≥1 米
    5:00-6:30  保持②  不要碰，等收敛

产出（默认 /tmp/movetest/<UTC戳>/）：
    samples.csv   每个采样一行（离线分析用；发我时给这个）
    live.log      人看的逐秒块，带阶段标记
    summary.txt   自动分段结论（屏幕上也会打一遍）

安全：只订阅 ROS 话题 + 只读模式文件；坐标变换复用 px4_live / vision_uplink，
      绝不另抄一份（上次的 bug 就是"抄一份再抄错"来的）。
"""
import argparse
import math
import os
import re
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import px4_live as P            # noqa: E402  同一套判据/变换，单一来源

CSV_COLS = ['iso', 'epoch', 't_rel', 'phase',
            'pn', 'pe', 'pd', 'ph',                       # 我们发的（视觉 ned）
            'ekf_n', 'ekf_e', 'ekf_d', 'fcyaw', 'vyaw', 'delta_deg',
            'pred_n', 'pred_e', 'dh', 'dv', 'resang_deg', 'ratio',
            'vx', 'vy', 'vz', 'odom_age', 'fc_age']

# 阶段提示词：写清楚"手要干什么"，因为判据依赖"挪远 + 之后别碰"
PROMPT = {
    '基准':  '保持飞机静止不动（相机对着有纹理的方向，别遮挡）',
    '平移①': '★ 动手：沿桌面把飞机【平移 ≥1 米】（贴桌面推、保持水平、别挡相机），到位后松手',
    '保持①': '↑ 松手后【不要碰飞机】，等飞控 EKF 重学收敛（Δ 会自己往下走，约 1~2 分钟）',
    '平移②': '★ 再动一次：换个方向（与刚才大致垂直）【平移 ≥1 米】，到位后松手',
    '保持②': '↑ 再次保持静止等收敛；本次是最后一段',
}


# ------------------------------------------------------------------ 采样
class Sampler(object):
    """只订阅 4 个镜像话题；每帧算出「我们发的 vs 飞控实际」的全套数字。"""

    def __init__(self):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        self.odom = None      # (t, ned, rpy_visual, twist)
        self.fc = None        # (t, x, y, z, rpy_fc)
        self.est = None       # (t, text)
        rospy.init_node('px4_move_test', anonymous=True)
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber('/px4/local_position_ned', PoseStamped, self._on_fc, queue_size=10)
        rospy.Subscriber('/px4/estimator_status', String, self._on_est, queue_size=10)

    def _on_odom(self, m):
        ned, rpy = P.vins_to_visual(m.pose.pose.position, m.pose.pose.orientation)
        tw = m.twist.twist.linear
        self.odom = (time.time(), ned, rpy, (tw.x, tw.y, tw.z))

    def _on_fc(self, m):
        import vision_uplink as V
        q = m.pose.orientation
        self.fc = (time.time(), m.pose.position.x, m.pose.position.y, m.pose.position.z,
                   V.q_to_rpy((q.x, q.y, q.z, q.w)))

    def _on_est(self, m):
        self.est = (time.time(), m.data)

    def snap(self, t_rel, phase):
        """取一帧 → dict（数据不全时数值列留 nan，仍写行，便于看断点）。"""
        now = time.time()
        r = dict(t_rel=t_rel, phase=phase,
                 iso=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(now)), epoch=now)
        for k in CSV_COLS:
            r.setdefault(k, float('nan'))
        if self.odom is None or self.fc is None:
            return r
        to, ned, rpy_v, tw = self.odom
        tf, fx, fy, fz, frpy = self.fc
        dyaw = P.wrap_pi(frpy[2] - rpy_v[2])
        px, py = P.rz_xy(ned[0], ned[1], dyaw)
        # 残余角：预期向量与 EKF 实际向量之间的夹角。稳态下 Δ ≈ 2·sin(残余角/2)·|p| ——
        # 2026-10-09 那次实测：位置误差能被这个角【完整解释】，说明没有多余的坐标系错位。
        resang = math.degrees(math.atan2(px * fy - py * fx, px * fx + py * fy))
        r.update(pn=ned[0], pe=ned[1], pd=ned[2], ph=math.hypot(ned[0], ned[1]),
                 ekf_n=fx, ekf_e=fy, ekf_d=fz,
                 fcyaw=math.degrees(frpy[2]), vyaw=math.degrees(rpy_v[2]),
                 delta_deg=math.degrees(dyaw), pred_n=px, pred_e=py,
                 dh=math.hypot(fx - px, fy - py), dv=fz - ned[2], resang_deg=resang,
                 vx=tw[0], vy=tw[1], vz=tw[2],
                 odom_age=now - to, fc_age=now - tf)
        if self.est is not None:
            m = re.search(r'pos_h_ratio=([-\d.eEnan]+)', self.est[1])
            if m:
                try:
                    r['ratio'] = float(m.group(1))
                except ValueError:
                    pass
        return r


# ------------------------------------------------------------------ 渲染
def fmt_row(r):
    """一帧 → 一小块人看的文字（同时进 live.log 和屏幕）。"""
    L = []
    L.append('── [%s  t=%+05.0fs] %s ──' % (r['phase'], r['t_rel'], r['iso'][11:19]))
    if not math.isfinite(r.get('ph', float('nan'))):
        L.append('   （还没数据：px4_imu_node 没起 / VINS 没起？）')
        return L
    L.append('  飞控  EKF N=%s E=%s D=%s   飞控 yaw=%s°'
             % (P._f(r['ekf_n']), P._f(r['ekf_e']), P._f(r['ekf_d']), P._f(r['fcyaw'], 7, 2)))
    L.append('  我们  视觉 N=%s E=%s D=%s  |p|=%s m  视觉 yaw=%s°  δ=%s°'
             % (P._f(r['pn']), P._f(r['pe']), P._f(r['pd']), P._f(r['ph'], 6),
                P._f(r['vyaw'], 7, 2), P._f(r['delta_deg'], 7, 2)))
    L.append('  预期  Rz(δ)·视觉  N=%s E=%s' % (P._f(r['pred_n']), P._f(r['pred_e'])))
    sym, why = P.verdict_horiz(r['dh'], r['ph'])
    L.append('  水平Δ %s m %s   垂直Δ %s m   残余角 %s°   pos_h_ratio=%s'
             % (P._f(r['dh'], 6), sym, P._f(r['dv'], 6), P._f(r['resang_deg'], 6, 2),
                P._f(r['ratio'], 6, 3)))
    L.append('    ↳ %s' % why)
    return L


# ------------------------------------------------------------------ 汇总
def _med(xs):
    xs = sorted(x for x in xs if math.isfinite(x))
    return xs[len(xs) // 2] if xs else float('nan')


def _find_moves(rows, thr=0.05, back_s=6.0, debounce_s=35.0):
    """从数据里自动找「被挪动」的时刻：|p| 在 back_s 秒里变化超过 thr。

    要比"窗口中位"而不是"相邻样本"—— 手推是连续动作，推得稳时逐个样本比会漏
    （2026-10-09 第一版就误报过"没检测到平移事件"）。"""
    fin = lambda v: isinstance(v, float) and math.isfinite(v)     # noqa: E731
    ev, last = [], -1e9
    for i in range(len(rows)):
        t = rows[i]['t_rel']
        if t - last < debounce_s:
            continue
        near = [r['ph'] for r in rows[max(0, i - 4):i + 1] if fin(r.get('ph'))]
        far = [r['ph'] for r in rows if t - back_s - 2.0 <= r['t_rel'] <= t - back_s and fin(r.get('ph'))]
        if len(near) >= 3 and len(far) >= 3 and abs(_med(near) - _med(far)) > thr:
            ev.append(i)
            last = t
    return ev


def summarize(rows, total_s, outdir):
    L = []
    good = [r for r in rows if math.isfinite(r.get('dh', float('nan')))]
    L.append('═══ 平移验证汇总 ═══   样本 %d / 有效 %d / 时长 %.0fs' % (len(rows), len(good), total_s))
    L.append('  文件：%s' % outdir)
    if not good:
        L.append('  ✗ 没有有效样本 —— 话题没数据（栈没起？）')
        return L

    # --- 按名义阶段 ---
    L.append('')
    L.append('  阶段     样本  |p|中位   水平Δ中位   Δ最小    Δ最大   Δ/|p|  残余角°  判词')
    phases = []
    for r in rows:
        if r['phase'] not in phases:
            phases.append(r['phase'])
    for ph in phases:
        g = [r for r in good if r['phase'] == ph]
        if len(g) < 3:
            continue
        php, dhm = _med([r['ph'] for r in g]), _med([r['dh'] for r in g])
        dhmin, dhmax = min(r['dh'] for r in g), max(r['dh'] for r in g)
        sym, _ = P.verdict_horiz(dhm, php)
        L.append('  %-8s %4d  %7.3f  %8.4f  %7.4f %7.4f  %6.3f  %6.2f   %s'
                 % (ph, len(g), php, dhm, dhmin, dhmax,
                    dhm / php if php > 1e-6 else float('nan'),
                    _med([r['resang_deg'] for r in g]), sym))

    # --- 挪动事件 + 收敛曲线（不看名义阶段，只看数据）---
    ev = _find_moves(rows)
    L.append('')
    if not ev:
        L.append('  没检测到平移事件（|p| 没跳变）—— 飞机没挪？判据仍不可信。')
    for k, i in enumerate(ev[:3]):
        t0 = rows[i]['t_rel']
        pre = [r for r in good if t0 - 10 <= r['t_rel'] < t0]
        pk = max([r['ph'] for r in good if t0 <= r['t_rel'] <= t0 + 60] or [float('nan')])
        L.append('── 第 %d 次挪动 @ t=%+.0fs（|p| %.3f → 之后 60s 内峰值 %.3f m）—— 收敛：'
                 % (k + 1, t0, _med([r['ph'] for r in pre]) if pre else float('nan'), pk))
        after = [r for r in good if r['t_rel'] >= t0]
        for j in range(0, min(len(after), 120), 10):
            w = after[j:j + 10]
            if len(w) < 5:
                break
            L.append('     t=%+05.0fs  |p|=%6.3f  水平Δ中位=%7.4f  最小=%7.4f'
                     % (w[0]['t_rel'], _med([r['ph'] for r in w]),
                        _med([r['dh'] for r in w]), min(r['dh'] for r in w)))

    # --- 末 60 秒结论 ---
    last = good[-min(len(good), 60 * 2):]        # 2Hz → 60s
    php, dhm = _med([r['ph'] for r in last]), _med([r['dh'] for r in last])
    sym, why = P.verdict_horiz(dhm, php)
    L.append('')
    L.append('★ 末 60 秒：|p| 中位 %.3f m，水平Δ 中位 %.4f m，Δ/|p| = %.3f，残余角中位 %.2f°'
             % (php, dhm, dhm / php if php > 1e-6 else float('nan'),
                _med([r['resang_deg'] for r in last])))
    L.append('  判词 %s  %s' % (sym, why))
    if php > 1e-6 and dhm > 0:
        L.append('  稳态下 Δ ≈ 2·sin(残余角/2)·|p| 应为 %.4f m —— 与实测一致就说明'
                 '位置误差能被"两侧偏航估计的微小不一致"完整解释，没有多余的坐标系错位。'
                 % (2 * math.sin(math.radians(_med([r['resang_deg'] for r in last])) / 2) * php))
    pmax = max(r['ph'] for r in good)
    if pmax < 0.5:
        L.append('  ⚠ 本次最大 |p| 只有 %.2f m：够排除"大角度错位"（那会给出 1.6×|p| 量级的 Δ），'
                 % pmax)
        L.append('    但不足以精确测残余小角度。有空挪到 ≥1.5 m 再跑一次，结论更硬。')
    elif pmax < 1.0:
        L.append('  · 最大 |p| = %.2f m：判据成立（非原点退化），再远一点会更有说服力。' % pmax)
    else:
        L.append('  ✓ 最大 |p| = %.2f m：远离原点，判据充分。' % pmax)
    return L


# ------------------------------------------------------------------ 主流程
def build_schedule(a):
    segs = [('基准', a.base), ('平移①', a.move), ('保持①', a.hold),
            ('平移②', a.move2), ('保持②', a.hold2)]
    return [(n, d) for n, d in segs if d > 0]


def main():
    ap = argparse.ArgumentParser(description='半自动平移验证采集（只读）')
    ap.add_argument('--hz', type=float, default=2.0, help='采样率（默认 2）')
    ap.add_argument('--base', type=float, default=30, help='基准静止秒数')
    ap.add_argument('--move', type=float, default=90, help='平移①窗口秒数')
    ap.add_argument('--hold', type=float, default=120, help='保持①秒数')
    ap.add_argument('--move2', type=float, default=60, help='平移②窗口秒数（0=跳过）')
    ap.add_argument('--hold2', type=float, default=90, help='保持②秒数（0=跳过）')
    ap.add_argument('--out', default='/tmp/movetest', help='输出根目录')
    ap.add_argument('--stamp', default=None, help='目录名（默认取当前 UTC 时刻）')
    ap.add_argument('--quiet', action='store_true', help='不往屏幕打逐秒块（仍写文件）')
    a = ap.parse_args()

    sched = build_schedule(a)
    total = sum(d for _, d in sched)
    stamp = a.stamp or time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    outdir = os.path.join(a.out, stamp)
    os.makedirs(outdir, exist_ok=True)
    f_csv = open(os.path.join(outdir, 'samples.csv'), 'w')
    f_live = open(os.path.join(outdir, 'live.log'), 'w')

    f_csv.write(','.join(CSV_COLS) + '\n')
    f_csv.flush()

    def say(s):
        print(s, flush=True)
        f_live.write(s + '\n')
        f_live.flush()

    say('# px4_move_test 开始  %s  总时长 %.0fs  输出 %s' % (stamp, total, outdir))
    say('# 阶段表：%s' % ' → '.join('%s %.0fs' % s for s in sched))

    S = Sampler()
    rows = []
    t0 = time.time()
    seg_i, seg_t0, said = 0, 0.0, None
    last_live = [0.0]      # 每 1s 一块
    interrupted = False
    try:
        while True:
            el = time.time() - t0
            while seg_i + 1 < len(sched) and el - seg_t0 >= sched[seg_i][1]:
                seg_t0 += sched[seg_i][1]
                seg_i += 1
            if el - seg_t0 >= sched[seg_i][1] and seg_i == len(sched) - 1:
                break
            name = sched[seg_i][0]
            if said != name:
                said = name
                left = sched[seg_i][1] - (el - seg_t0)
                say('\a\n' + '=' * 68)
                say('【%s】剩 %.0fs   %s' % (name, left, PROMPT.get(name, '')))
                say('=' * 68)
            r = S.snap(el, name)
            rows.append(r)
            f_csv.write(','.join(
                ('%.6f' % r[c]) if isinstance(r.get(c), float) and math.isfinite(r.get(c, float('nan')))
                else ('' if isinstance(r.get(c), float) else str(r.get(c)))
                for c in CSV_COLS) + '\n')
            if el - last_live[0] >= 1.0:
                last_live[0] = el
                blk = fmt_row(r)
                f_live.write('\n'.join(blk) + '\n')
                f_live.flush()
                if not a.quiet:
                    print('\n'.join(blk), flush=True)
            f_csv.flush()
            time.sleep(max(0.0, 1.0 / a.hz))
    except KeyboardInterrupt:
        interrupted = True
        say('\n# 收到 Ctrl-C，用已采到的数据出结论')

    total_s = time.time() - t0
    lines = summarize(rows, total_s, outdir)
    say('\n'.join(lines))
    with open(os.path.join(outdir, 'summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    say('\n# 数据目录：%s  （samples.csv / live.log / summary.txt）%s'
        % (outdir, '  [提前中断]' if interrupted else ''))
    f_csv.close()
    f_live.close()


if __name__ == '__main__':
    main()
