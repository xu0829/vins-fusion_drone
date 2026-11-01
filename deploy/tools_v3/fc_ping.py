#!/usr/bin/env python3
"""探一下飞控在不在：收到心跳退出码 0，没收到退出码 1。

给 wait_fc_then_start_imu.sh 当探针用。刻意保持极简——只做一件事，
免得探针自己出问题反而更难查。

★ 2026-09-28 修正：原来是一个裸的 `except Exception: sys.exit(1)`，把
  【所有】异常都报成"飞控不在"。于是下面这些完全不同的情况长得一模一样：
      · 飞控真死了（没心跳）
      · 串口打不开 / 被别的进程占着
      · 飞控正在 USB 重枚举，/dev/ttyACM0 还不存在或刚重建
      · pymavlink 自己出问题
  实战后果：B 轮启动器的第 0 步报「✗ 飞控没心跳」并拒绝启动，而同一时刻
  /imu 正以 195 Hz 在发、fc_ping 单独跑也返回 0 —— 我据此追错了方向一轮。
  ⇒ 退出码语义【保持不变】（0=有心跳，1=没有），只把原因打到 stderr，
    这样调用方行为不变，但下次出问题能看见真正的原因。
"""
import sys
import traceback

from pymavlink import mavutil

# ★ 2026-09-28 第三处修正（也是最关键的一处）：把 pymavlink 的存档 bug 挡掉。
#
# 症状：wait_heartbeat() 抛
#     TypeError: 'NoneType' object does not support item assignment
#   位置 mavutil.py add_message() 的 messages[mtype]._instances[instance_value] = msg
#   —— messages[mtype] 那条消息的 _instances 是 None。
#
# 为什么它会要了探针的命：wait_heartbeat() 会把流里【每一条】消息都解出来并
# post 一遍，只要任意一条踩中这个坑，异常就一路冒到顶，把整个等待打死 ——
# 哪怕心跳在几毫秒后就到了。实测这条几乎【每次重试都犯】（重试 8 秒全灭），
# 因为它跟的是某个飞控持续在发的消息类型，不是随机时序。
#
# 而这个错误跟我们想知道的事【毫无关系】：我们只想等一个心跳，消息存档坏了
# 不重要。所以在这里把它降级成"这条消息不存档"，让 recv_msg 正常返回，
# 心跳该被看见就被看见。★ 只作用于本探针进程，不改任何库文件。
_orig_add_message = mavutil.add_message


def _tolerant_add_message(messages, mtype, msg):
    try:
        _orig_add_message(messages, mtype, msg)
    except Exception:  # noqa: BLE001  存档失败无所谓，绝不能因此打断等心跳
        try:
            messages[mtype] = msg
        except Exception:  # noqa: BLE001
            pass


mavutil.add_message = _tolerant_add_message

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'


def _fail(reason):
    """退出码 1，但先把原因说清楚——不要再让不同的病因长成同一句话。"""
    print("fc_ping: %s" % reason, file=sys.stderr)
    sys.exit(1)


try:
    m = mavutil.mavlink_connection(PORT, baud=115200, timeout=2,
                                   autoreconnect=False)
except Exception as e:  # noqa: BLE001
    print("fc_ping: 打开 %s 失败：%s: %s" % (PORT, type(e).__name__, e),
          file=sys.stderr)
    traceback.print_exc(file=sys.stderr)
    sys.exit(1)

# ★ 2026-09-28 第二处修正：重试，而不是"第一次出错就判死"。
#
# 实测（fc_ping_rep.sh，每种条件 6 次采样）：
#     桥在跑 : 过 3 / 败 3     桥停掉 : 过 3 / 败 3
#   失败率【完全一样】⇒ 抢串口不是主因。真正的两种死法：
#
#   (1) TypeError: 'NoneType' object does not support item assignment
#       位置 pymavlink/mavutil.py add_message() -> messages[mtype]._instances[..]
#       ★ 这跟飞控一点关系都没有。wait_heartbeat() 会把流里【每一条】消息都
#         解出来并 post 一遍，只要其中任意一条踩中 pymavlink 这个坑，异常就
#         一路冒到顶，把整个调用打死 —— 哪怕心跳就在几毫秒后到。串口空着也照样犯。
#   (2) SerialException: device reports readiness to read but returned no data
#       这个才真是抢串口（bridge 在跑时才出现）：select 说可读，但字节已被
#       另一个读者（px4_imu_node）取走。
#
# 两种都是【瞬时】故障，下一轮读多半就好了。所以这里改成在截止时间内反复重试，
# 只在【整整 8 秒一次都没成】时才报"没心跳"——那才是真的没心跳。
try:
    import time as _time
    _deadline = _time.time() + 8.0
    hb = None
    _errs = []
    while hb is None and _time.time() < _deadline:
        try:
            hb = m.wait_heartbeat(timeout=1.5)
        except Exception as e:  # noqa: BLE001  瞬时故障，记下来继续等
            _errs.append("%s: %s" % (type(e).__name__, e))
            _time.sleep(0.2)
    if hb is None and _errs:
        print("fc_ping: 8 秒内重试 %d 次都没等到心跳。最后一次错误：%s"
              % (len(_errs), _errs[-1]), file=sys.stderr)
        print("fc_ping: 出现过的错误种类：%s"
              % sorted(set(e.split(':')[0] for e in _errs)), file=sys.stderr)
except Exception as e:  # noqa: BLE001
    try:
        m.close()
    except Exception:  # noqa: BLE001
        pass
    print("fc_ping: 串口开着但等心跳时出错：%s: %s" % (type(e).__name__, e),
          file=sys.stderr)
    traceback.print_exc(file=sys.stderr)
    sys.exit(1)

try:
    m.close()
except Exception:  # noqa: BLE001
    pass

if not hb:
    _fail("%s 8 秒内没有心跳（串口能打开，所以更像是飞控侧或 MAVLink 流没配）"
          % PORT)

sys.exit(0)
