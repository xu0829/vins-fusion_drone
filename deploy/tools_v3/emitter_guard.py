#!/usr/bin/env python3
"""发射器快速守卫 —— 常驻订阅，毫秒级反应，补 launcher 里 20 秒慢看门狗的漏。

为什么需要（2026-09-28 19:36 实测）：
  相机 USB 重枚举 → librealsense 把 emitter_enabled 重置回 1（红外点阵打开）
  → 散斑污染 → 误匹配 → 垃圾视觉旋转 → VINS 状态被甩飞。
  而 launcher 的 emitter_watchdog 在 `start_vins_drone.sh:78` 里 `sleep 20`
  （加 emitter_state 自己的 timeout 5），**最坏暴露 25 秒**。

用法（容器内）：
    source /opt/ros/noetic/setup.bash && python3 -u emitter_guard.py > /tmp/vins_drone/emitter_guard.log 2>&1

★ 两个亲身踩过的坑，别再犯：
  1) **`emitter_enabled` 是 int，不是 bool！** 它在 `dynamic_reconfigure/Config.ints` 里，
     不在 `.bools` 里（`.bools` 里是 emitter_on_off / emitter_always_on 这些**别的**参数）。
     我第一版只扫 `.bools`，于是这个守卫从头到尾一次都没触发过 —— 而我还差点拿它的
     "没记录" 当成 "发射器没被勾开" 的证据。**仪表必须先用对照实验验活**（手动置 1，
     看它响不响），验活之前它的一切沉默都不算证据。见 [[instrument-must-touch-the-measured-signal]]。
  2) 只走 stdout 由调用方重定向；脚本内部再 open(LOG,"a") 写同一个文件会和 shell 的 `>`
     抢文件偏移量，日志当场串行错乱。

本脚本只写 emitter_enabled 这一个参数，不碰其它任何参数。
"""
import time

import rospy
from dynamic_reconfigure.msg import Config, IntParameter
from dynamic_reconfigure.srv import Reconfigure

NODE = "/camera/stereo_module"
PARAM = "emitter_enabled"          # ★ int 型，在 Config.ints 里
HEARTBEAT = 60.0                   # 秒；有心跳才证明仪表活着

rospy.init_node("emitter_guard", anonymous=True)


def w(msg):
    print("[guard] %s %s" % (time.strftime("%F %T"), msg), flush=True)


w("等 %s/set_parameters 出现（相机节点可能还没起）..." % NODE)
try:
    rospy.wait_for_service(NODE + "/set_parameters", timeout=180)
except rospy.ROSException:
    w("180 秒等不到服务，退出")
    raise SystemExit(1)

setter = rospy.ServiceProxy(NODE + "/set_parameters", Reconfigure)
w("服务就绪，开始常驻监听（每次变化都记录，60 秒一条心跳）")

stat = {"last": None, "n": 0, "msgs": 0}


def cb(msg):
    stat["msgs"] += 1
    for it in msg.ints:                     # ★ 这里是 ints，不是 bools
        if it.name != PARAM:
            continue
        if stat["last"] != it.value:
            w("读到 %s = %d（累计第 %d 条消息）" % (PARAM, it.value, stat["msgs"]))
            stat["last"] = it.value
        if it.value != 0:
            try:
                setter(Config(ints=[IntParameter(name=PARAM, value=0)]))
                stat["n"] += 1
                w("★ 发射器被勾开(%d) → 已关（累计 %d 次）" % (it.value, stat["n"]))
            except rospy.ServiceException as e:
                w("关闭失败：%s" % e)


rospy.Subscriber(NODE + "/parameter_updates", Config, cb, queue_size=50)


def beat(_ev):
    w("心跳：收到 %d 条消息，最近读到的 %s=%s"
      % (stat["msgs"], PARAM, stat["last"]))


rospy.Timer(rospy.Duration(HEARTBEAT), beat)
rospy.spin()
