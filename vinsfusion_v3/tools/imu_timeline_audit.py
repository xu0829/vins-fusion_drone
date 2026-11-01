#!/usr/bin/env python3
"""/imu 时间轴体检：采样率为什么掉、mapper 的 scale 是真的还是假的。

背景：px4_imu_node.py 用 ImuClockMapper 把飞控时间轴映射到宿主时钟，报出的
scale 从 -564 ppm 一路漂到 -919 ppm。这是两种截然不同的病，但 mapper 分不出来：

  真时钟差  -> 飞控晶振和宿主晶振频率不同，arrival_offset 随时间【线性】增长。
               scale 是对的，应该保留。
  传输延迟  -> 串口/USB 排队、丢包重传，arrival_offset 会【游走/跳变】。
               scale 是假的，是在给不存在的东西做拉伸，应该用 IMU_CLOCK_MODE=lower
               把 scale 锁成 1.0。

判据就在 arrival_offset 去趋势后的残差上：线性 => 残差极小；游走 => 残差大且不规则。

同时体检采样率：sensor_time 的间隔直方图能区分
  "飞控真的采慢了"（间隔整体变大、仍旧均匀）
  "飞控漏发了"    （间隔是基准周期的整数倍堆叠）

用法（容器内）:
    python3 imu_timeline_audit.py /tmp/imu_raw.csv
"""
import sys

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/imu_raw.csv'
d = np.genfromtxt(path, delimiter=',', names=True)

st = d['sensor_time']      # 飞控采样时刻（飞控时钟）
at = d['arrival_time']     # 宿主收到时刻（宿主时钟）
n = len(st)
print('样本 %d 条，跨度 %.1f 秒（按飞控时间轴）' % (n, st[-1] - st[0]))
print()

# ---------- A. sensor_time 间隔直方图：漏发还是真慢 ----------
dt = np.diff(st) * 1000.0                     # ms
base = np.median(dt)
print('=== A. 飞控采样间隔（中位 %.3f ms -> %.1f Hz）===' % (base, 1000.0 / base))
# 按"基准周期的几倍"归堆，能一眼看出漏发
mult = np.round(dt / base).astype(int)
print('  倍率   间隔(ms)      条数     占比')
for k in range(1, 7):
    m = mult == k
    if m.sum() == 0:
        continue
    print('   x%-3d  %8.3f  %9d   %5.1f%%' % (k, dt[m].mean(), m.sum(), 100.0 * m.sum() / len(dt)))
big = mult > 6
if big.sum():
    print('   >x6   %8.3f  %9d   %5.1f%%   <- 大洞' % (dt[big].mean(), big.sum(),
                                                     100.0 * big.sum() / len(dt)))
lost = (mult - 1).sum()
print('  => 等价漏发 %d 条（%.1f%%）；飞控【发出】速率 %.1f Hz，'
      '我们【收到】%.1f Hz'
      % (lost, 100.0 * lost / (n + lost), 1000.0 / base, 1000.0 / dt.sum() * (n - 1)))
print()

# ---------- B. 速率随时间：什么时候掉的 ----------
print('=== B. 接收速率随时间（每 120 秒一桶）===')
print('   宿主时刻            收到条数   速率Hz')
t0 = at[0]
edges = np.arange(t0, at[-1] + 120.0, 120.0)
idx = np.searchsorted(at, edges)
for i in range(len(edges) - 1):
    c = idx[i + 1] - idx[i]
    if c == 0:
        continue
    import datetime
    ts = datetime.datetime.fromtimestamp(edges[i]).strftime('%H:%M:%S')
    bar = '#' * int(c / 60)
    print('   %s   %8d   %6.1f  %s' % (ts, c, c / 120.0, bar))
print()

# ---------- C. arrival_offset 去趋势残差：决定性判据 ----------
# arrival_offset 含一个巨大的常数（两个时钟的纪元差），只关心它的变化。
x = st - st[0]
y = (at - at[0]) - x                       # 相对漂移，秒
k, b = np.polyfit(x, y, 1)
res = y - (k * x + b)
print('=== C. arrival_offset 漂移（决定性）===')
print('  总漂移 %.1f ms / %.0f 秒  ->  等效 %+.1f ppm'
      % (y[-1] * 1000, x[-1], k * 1e6))
print('  去趋势残差: 标准差 %.2f ms，最大 %.2f ms，峰峰 %.2f ms'
      % (res.std() * 1000, np.abs(res).max() * 1000,
         (res.max() - res.min()) * 1000))
print('  最近 120 秒的残差标准差 %.2f ms'
      % (res[x > x[-1] - 120].std() * 1000))
print()
print('  判读:')
print('    残差 < 2ms 且平滑      => 真时钟差，scale 该保留')
print('    残差与总漂移同量级     => 前半是传输延迟，scale 是假的')
print('    残差里有 >50ms 的台阶  => 串口卡顿，scale 更是假的')
print()
# 残差的粗粒度走势，能直接看出是线性爬升还是游走
print('  残差走势（每 10% 时间取一点，单位 ms）:')
step = max(1, n // 12)
print('   ', '  '.join('%+.1f' % (r * 1000) for r in res[::step][:12]))
