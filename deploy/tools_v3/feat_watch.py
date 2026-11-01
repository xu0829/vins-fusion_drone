#!/usr/bin/env python3
"""实时显示两路相机各能提取多少个角点 —— 把"VINS 看不看得见"变成看得见的数字。

为什么需要：VINS 发散时，人会本能地怀疑代码、标定、供电。但这一晚的实测是
**输入完美、算法却炸**，真正的变量是"镜头前有没有东西可看"。
这个仪表把那个变量直接打进终端，开灯/换视角时数字会当场变。

角点用 Shi-Tomasi，参数与 VINS 前端同量级（0.01 / 30px / 上限 150），
所以这个数字可以和记忆里的基线直接比：
    健康静止基线 83~108 / 150      ← 正常
    本晚实测     63 / 32           ← 视觉饿死
    发射器开着   42 / 150          ← 散斑污染（更差）

用法（容器内）：
    source /opt/ros/noetic/setup.bash && python3 feat_watch.py [秒数]
不传秒数就一直跑，Ctrl-C 停。
"""
import sys
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

# ★ 不用 cv_bridge。香橙派镜像（v2 和 v3 都一样）里 cv_bridge 的 Python 绑定
#   导入即抛 SystemError: initialization of cv_bridge_boost raised unreported exception
#   —— 是镜像打包缺陷，不是本次部署引入的（C++ 的 libcv_bridge.so 正常，
#   vins_node 不受影响，已实测跑到 init begins）。
#   而 infra*_rect_raw 是 mono8，本来就是字节数组，直接 reshape 即可，不需要解码器。


def to_mono8(m):
    """把 mono8 的 Image 直接变成 numpy 数组，绕开 cv_bridge。"""
    if m.encoding not in ("mono8", "8UC1"):
        return None
    # 按 step（行长）reshape 再裁到 width —— 相机可能带行填充
    return np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]

LIMIT = 150
# 判据（来自记忆 feature-collapse-is-camera-usb-precursor / ir-emitter-*）
GOOD, WARN = 80, 50

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 1e9
rospy.init_node("feat_watch", anonymous=True)

latest = {}


def mk(k):
    def cb(m):
        try:
            img = to_mono8(m)
            if img is not None:
                latest[k] = img
        except Exception:  # noqa: BLE001
            pass
    return cb


for name, topic in (("infra1", "/camera/infra1/image_rect_raw"),
                    ("infra2", "/camera/infra2/image_rect_raw")):
    rospy.Subscriber(topic, Image, mk(name), queue_size=2)

t0 = time.time()
while not rospy.is_shutdown() and time.time() - t0 < dur:
    row = []
    for k in ("infra1", "infra2"):
        img = latest.get(k)
        if img is None:
            row.append(f"{k}: --")
            continue
        pts = cv2.goodFeaturesToTrack(img, maxCorners=LIMIT, qualityLevel=0.01,
                                      minDistance=30, blockSize=3)
        n = 0 if pts is None else len(pts)
        mark = "✓" if n >= GOOD else ("·" if n >= WARN else "✗")
        row.append(f"{k}: {n:3d}/{LIMIT} {mark} 亮度{img.mean():5.1f}")
    print(f"   [{time.strftime('%H:%M:%S')}] " + "   ".join(row), flush=True)
    time.sleep(2.0)
