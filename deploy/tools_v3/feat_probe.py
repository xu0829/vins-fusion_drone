#!/usr/bin/env python3
"""infra1 角点数为什么上不去 —— 把"门限"和"画面"分开量，别猜。

动机：香橙派这一轮 infra2 能到 84~94，infra1 卡在 74~77 上不去。
抓帧看画面（/tmp/pi_frames/n1.png）纹理明明是有的：书架、书脊、键帽、线缆都在。
所以"没东西可看"这个解释不够用了，得换一个能验证的。

机制（可验证的假设）：qualityLevel=0.01 不是绝对阈值，是【相对画面最强角点】的——
    实际门限 = 0.01 × max(全图 minEigenVal)
画面里只要有一块过曝死白（本帧 7% 像素 = 255），它的边缘就是一个极强阶跃，
把 max 抬上去，门限整体跟着涨，中等强度的角点（书脊、键帽、木纹）成批被刷掉。
—— 这跟笔记本"对着白墙那轮 72 个角点"是两回事：那轮是真的没纹理，这轮是纹理被门限压住了。

★ 和 goodFeaturesToTrack 的 mask 参数配合正好能做成对照实验：
  OpenCV 内部 `minMaxLoc(eig, 0, &maxVal, 0, 0, _mask)` 会把 mask 带进去，
  所以【遮掉死白区再算一次】= 同一个函数、同一个 qualityLevel，只是不让死白区定门限。
  两者之差就是"死白到底压掉了多少真实纹理"。

另存 infra1_corners.png：红圈=现状角点，绿=过曝区，蓝点=遮掉过曝后才捞回来的。
看这张图比看任何数字都快。

用法（容器内）：
    source /opt/ros/noetic/setup.bash && python3 feat_probe.py [/tmp/vins_pi]
"""
import sys
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

LIMIT, QL, MIND, BS = 150, 0.01, 30, 3   # 与 VINS 前端 / feat_watch.py 同量级
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp"


def to_mono8(m):
    """mono8 直接变成 numpy，绕开镜像里坏掉的 cv_bridge（见 feat_watch.py 注释）。"""
    if m.encoding not in ("mono8", "8UC1"):
        return None
    return np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]


def grab(topic, timeout=10.0):
    box = {}

    def cb(m):
        if "g" not in box:
            try:
                g = to_mono8(m)
                if g is not None:
                    box["g"] = g
            except Exception:  # noqa: BLE001
                pass

    rospy.Subscriber(topic, Image, cb, queue_size=1)
    t0 = time.time()
    while "g" not in box and time.time() - t0 < timeout and not rospy.is_shutdown():
        time.sleep(0.05)
    return box.get("g")


rospy.init_node("feat_probe", anonymous=True)
img = grab("/camera/infra1/image_rect_raw")
if img is None:
    print("✗ 没收到 infra1（话题没数据，或者不是 mono8）")
    sys.exit(1)

h, w = img.shape
print(f"infra1 {w}x{h}  亮度 min={img.min()} 均值={img.mean():.1f} "
      f"中位={np.median(img):.0f} max={img.max()}")

# ---------------- A. 现在这套参数的门限在哪儿 ----------------------------
eig = cv2.cornerMinEigenVal(img, BS, 3)
emax_full = float(eig.max())
yx = np.unravel_index(int(eig.argmax()), eig.shape)
thr = QL * emax_full

sat = img >= 250
sat_d = cv2.dilate(sat.astype(np.uint8), np.ones((41, 41), np.uint8)) > 0
eig_nosat = eig.copy()
eig_nosat[sat_d] = 0
emax_nosat = float(eig_nosat.max())
thr_should = QL * emax_nosat

print(f"\nA. 门限（qualityLevel={QL} 是【相对】画面最强角点的）")
print(f"   全图最强响应 maxEig = {emax_full:9.1f}   在 (x={yx[1]:3d}, y={yx[0]:3d})")
print(f"   ⇒ 实际门限 = {QL} × {emax_full:.1f} = {thr:.3f}")
print(f"   过曝像素(≥250)：{int(sat.sum()):6d} 个 = {100.0 * sat.sum() / img.size:4.1f}% 画面")
print(f"   最强的【非过曝】响应   = {emax_nosat:9.1f}")
print(f"   ⇒ 门限本该是 {thr_should:.3f}，现在高了 "
      f"{thr / max(thr_should, 1e-9):.1f} 倍")
print(f"   最强点是" + ("贴着过曝区  ← 门限就是它抬的" if sat_d[yx] else "不在过曝区附近"))

# ---------------- B. 角点数：现状 vs 遮掉过曝 ----------------------------
pts = cv2.goodFeaturesToTrack(img, maxCorners=LIMIT, qualityLevel=QL,
                              minDistance=MIND, blockSize=BS)
n0 = 0 if pts is None else len(pts)

mask = np.full((h, w), 255, np.uint8)
mask[sat_d] = 0
pts_m = cv2.goodFeaturesToTrack(img, maxCorners=LIMIT, qualityLevel=QL,
                                minDistance=MIND, blockSize=BS, mask=mask)
nm = 0 if pts_m is None else len(pts_m)

print(f"\nB. 角点数")
print(f"   现状（VINS 参数，原样）        ：{n0:3d}/{LIMIT}")
print(f"   遮掉过曝区（同一 qualityLevel）：{nm:3d}/{LIMIT}")
if nm > n0:
    print(f"   ⇒ 死白抬高了门限，压掉了 {nm - n0} 个角点"
          f"（占可用纹理的 {100.0 * (nm - n0) / max(nm, 1):.0f}%）")
    print(f"   ⇒ 结论：**画面里有纹理，是过曝把门限抬起来把它们刷掉了**。"
          f"把强光源移出画面即可，不用换视角。")
else:
    print(f"   ⇒ 遮掉过曝区也捞不回来 ⇒ 过曝不是主因")
    print(f"   ⇒ 那就是画面本身对比度低（红外下暗对暗），或者纹理尺度大得超过 minDistance={MIND}px")
    print(f"   ⇒ 得换视角找实际有对比度的东西，光开灯不一定够")

# ---------------- C. 角点落在画面的哪几块 --------------------------------
grid = np.zeros((3, 3), int)
if pts is not None:
    for p in pts.reshape(-1, 2):
        grid[min(2, int(3 * p[1] / h)), min(2, int(3 * p[0] / w))] += 1
print(f"\nC. 现状 {n0} 个角点的分布（3×3，行=上中下，列=左中右）")
for r in range(3):
    print("      " + "  ".join(f"{grid[r, c]:3d}" for c in range(3)))
nz = int((grid > 0).sum())
print(f"   有角点的格子：{nz}/9" + ("（分布还行）" if nz >= 6 else "  ← 集中在少数几块，视觉退化"))

# ---------------- D. 存图 -------------------------------------------------
vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
vis[sat] = (0, 255, 0)
if pts is not None:
    for p in pts.reshape(-1, 2).astype(int):
        cv2.circle(vis, (int(p[0]), int(p[1])), 7, (0, 0, 255), 2)
if pts_m is not None:
    for p in pts_m.reshape(-1, 2).astype(int):
        cv2.circle(vis, (int(p[0]), int(p[1])), 3, (255, 0, 0), -1)
fn = f"{OUT}/infra1_corners.png"
cv2.imwrite(fn, vis)
print(f"\nD. 存图 {fn}")
print(f"   绿=过曝区(≥250)  红圈=现状角点  蓝点=遮掉过曝后捞回来的")
