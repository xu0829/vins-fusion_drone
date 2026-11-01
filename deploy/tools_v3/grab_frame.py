#!/usr/bin/env python3
"""抓一帧 infra1/infra2，量亮度和角点数 —— 直接回答"相机现在看得见东西吗"。

为什么需要：IMU 和相机的时间轴都验干净了、飞机也确认静止，VINS 却照样
飞到几公里 —— 那剩下的唯一变量就是"视觉前端有没有料"。

这台相机我们**故意关掉了 IR 发射器**（开着会有散斑污染），所以它完全靠
环境光。房间里关灯 ⇒ 图像全黑 ⇒ 特征为零 ⇒ VINS 只剩 IMU 可用，
位置必然飘走。记忆里"测前开灯"说的就是这个坑。

角点用 Shi-Tomasi，参数刻意跟 VINS 的前端一个量级（0.01 / 30px），
这样"角点数"才真的能代表 VINS 能看到多少 —— 换个门限量的就是别的东西了。
"""
import sys

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

# ★ 不用 cv_bridge：香橙派镜像里它的 Python 绑定导入即抛 SystemError
#   （打包缺陷，v2/v3 都有；C++ 的 libcv_bridge.so 正常，vins_node 不受影响）。
#   mono8 的 raw 图本身就是字节数组，直接 reshape，不需要解码器。
out = sys.argv[1] if len(sys.argv) > 1 else "/root/xu_drone/live"
rospy.init_node("grab_frame", anonymous=True)

for name, topic in (("infra1", "/camera/infra1/image_rect_raw"),
                    ("infra2", "/camera/infra2/image_rect_raw")):
    try:
        msg = rospy.wait_for_message(topic, Image, timeout=10)
    except Exception as e:  # noqa: BLE001
        print(f"  {name}: 拿不到帧（{e}）")
        continue
    if msg.encoding not in ("mono8", "8UC1"):
        print(f"  {name}: 编码是 {msg.encoding}，不是 mono8，跳过")
        continue
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
    pts = cv2.goodFeaturesToTrack(img, maxCorners=150, qualityLevel=0.01,
                                  minDistance=30, blockSize=3)
    n = 0 if pts is None else len(pts)
    hist = np.histogram(img, bins=16, range=(0, 256))[0]
    print(f"  {name}: {img.shape[1]}x{img.shape[0]}  亮度 min={img.min()} "
          f"均值={img.mean():.1f} 中位={int(np.median(img))} max={img.max()}")
    print(f"      像素分布(16档 0→255): {' '.join(str(v) for v in hist)}")
    print(f"      Shi-Tomasi 角点(上限150, 与 VINS 同量级): {n}")
    p = f"{out}/{name}_now.png"
    cv2.imwrite(p, img)
    print(f"      已存 {p}")
