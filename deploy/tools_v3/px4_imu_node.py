#!/usr/bin/env python3
"""ROS1 节点：把 PX4 的 HIGHRES_IMU 直接变成 /imu 话题（给 VINS-Fusion 用）。

为什么不用 mavros：
  · 容器是 ROS1 Noetic，mavros 得装一整套 + 配 FCU URL，还得 relay 一次
    （/mavros/imu/data_raw → /imu），多一层就多一处时间戳被改的机会。
  · 香橙派那套桥（vio_bridge_combined.py）本身就是直接用 pymavlink 读串口的，
    这里走同一条路，将来往香橙派搬的时候行为一致。

两件必须做对的事（做错了是静默错，不报错）：

1. 时间戳用【飞控采样时钟】映射，不是【到达时刻】。
   实测：接收 dt 中位 6.067ms / 采样 dt 中位 4.552ms —— 两者不一样，说明样本是
   成簇到达的。直接拿到达时刻打戳，等于把串口+调度的抖动灌进预积分。
   映射策略直接搬香橙派的 ImuClockMapper（mode='slow'），一行不改。

2. 坐标轴要做 FRD→FLU 翻转（y、z 取负）。
   PX4 的 HIGHRES_IMU 是 FRD（x前/y右/z下）；标定包的 /imu 实测静止时
   accel.z = +9.775，是 FLU。Kalibr 的外参就是在这个 FLU body 系下标出来的，
   不翻的话 body_T_cam 会静默对不上。

用法:
    python3 px4_imu_node.py [/dev/ttyACM0] [baud=115200]
    环境变量 IMU_CLOCK_MODE 可切 'slow'(默认) / 'lower' / 'ema005'
"""
import os
import sys
import time
import threading

import rospy
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imu_clock_mapper import ImuClockMapper  # noqa: E402

# ★ 必须在 import pymavlink 之前：切 MAVLink 2。理由（实测，见 VISION_UPLINK_CN.md）：
#   102 号消息的 covariance 是【扩展字段】，MAVLink 1 压根带不出去 —— PX4 收到会补零，
#   若 EKF2_EV_NOISE_MD=1 就是"零方差 = 无限自信"，比不传更糟。
#   升 v2 只改帧格式（头 6→10 字节），IMU 的载荷字段一个都没变；而且 v2 的解析器
#   两种帧都能收，就算飞控继续回 v1 帧也不受影响。要退回：export MAVLINK20=0
os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil  # noqa: E402

# 视觉上行（VINS odometry → PX4）+ 飞控遥测镜像。跑在本进程里是硬要求：
# /dev/ttyACM0 的字节只给一个 reader，另起进程会和这里抢字节。
# 用 try/except 包住 —— 这个模块出任何问题都不许挡到 IMU 桥。VISION_UPLINK=0 可整体关。
try:
    if os.environ.get('VISION_UPLINK', '1') != '0':
        from vision_uplink import VisionUplink  # noqa: E402
    else:
        VisionUplink = None
except Exception as _e:  # noqa: BLE001
    VisionUplink = None
    print('[imu] vision_uplink 加载失败（IMU 不受影响）: %s' % _e, file=sys.stderr)

PUB_TOPIC = rospy.get_param('~topic', '/imu')
FRAME_ID = rospy.get_param('~frame_id', 'imu')
PORT = sys.argv[1] if len(sys.argv) > 1 else rospy.get_param('~port', '/dev/ttyACM0')
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else int(rospy.get_param('~baud', 115200))
RATE_HZ = float(rospy.get_param('~rate', 200.0))
CLOCK_MODE = os.environ.get('IMU_CLOCK_MODE', rospy.get_param('~clock_mode', 'slow'))

# HIGHRES_IMU = 234；200Hz = 5000us
MSG_ID_HIGHRES_IMU = 234
INTERVAL_US = int(1e6 / RATE_HZ)


class Px4ImuNode:
    def __init__(self):
        self.pub = rospy.Publisher(PUB_TOPIC, Imu, queue_size=200)
        self.clock = ImuClockMapper(mode=CLOCK_MODE, nominal_rate=RATE_HZ)
        self.master = None
        self.n_pub = 0
        self.n_short_dt = 0
        self.last_scale_ppm = 0.0
        self.last_stamp = None
        self.t_start = time.time()
        self.t_last_report = time.time()
        self.lock = threading.Lock()
        self._stop = threading.Event()

        # 视觉上行（与 IMU 共用一条 MAVLink 连接；所有串口 I/O 都在主循环里做）
        self.uplink = None
        self.vision_ref = None   # (fc_usec, host_stamp, scale_ppm)：最近一条 IMU 的时钟参考对
        if VisionUplink is not None:
            try:
                self.uplink = VisionUplink()
            except Exception as e:  # noqa: BLE001
                rospy.logwarn('视觉上行初始化失败（IMU 不受影响）: %s', e)
                self.uplink = None

        # 原始三元组落盘：飞控采样时刻、到达时刻、映射后时间戳。
        # 事后才能判断 offset 的漂移到底是"两个时钟速率差"（线性）还是
        # "串口/USB 传输延迟"（游走、跳变）—— 两者在 mapper 眼里长得一样，
        # 但只有前者该被当成 scale 去拉伸 IMU 时间轴。
        self.csv = None
        csv_path = os.environ.get('IMU_CSV', '')
        if csv_path:
            self.csv = open(csv_path, 'w', buffering=1)
            self.csv.write('sensor_time,arrival_time,stamp,scale_ppm\n')
            rospy.loginfo('原始时间对落盘: %s', csv_path)

    # ---------- 串口 ----------
    def connect(self):
        rospy.loginfo('连接 %s @ %d ...', PORT, BAUD)
        self.master = mavutil.mavlink_connection(PORT, baud=BAUD, timeout=3,
                                                 autoreconnect=False)
        if not self.master.wait_heartbeat(timeout=10):
            raise IOError('10 秒内没收到心跳 —— 飞控没跑固件，或串口被别的程序占着')
        rospy.loginfo('心跳到达: sysid=%s compid=%s',
                      self.master.target_system, self.master.target_component)
        self.request_rates()

    def request_rates(self):
        """MAV_CMD_SET_MESSAGE_INTERVAL 把 HIGHRES_IMU 拉到 200Hz。

        只在飞控内存里生效，断电即恢复，不改固件、不落盘。
        实测这一条把 HIGHRES_IMU 从 133.3Hz 提到 152.0Hz，并且空洞(>50ms)
        从 8 个降到 0 个 —— 是必须发的。
        """
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                MSG_ID_HIGHRES_IMU, INTERVAL_US, 0, 0, 0, 0, 0)
            rospy.loginfo('已请求 HIGHRES_IMU @ %.0fHz', RATE_HZ)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn('SET_MESSAGE_INTERVAL 失败: %s', e)

    def reconnect(self, reason):
        rospy.logwarn('重连（%s）...', reason)
        try:
            if self.master is not None:
                self.master.close()
        except Exception:  # noqa: BLE001
            pass
        self.master = None
        time.sleep(1.0)
        try:
            self.connect()
            # 换了串口会话，时间基准要重新建立
            self.clock.reset(reason='serial_reconnect')
            self.vision_ref = None
            if self.uplink is not None:
                self.uplink.on_reconnect()   # 遥测流/参数请求在飞控侧也随会话没了
            return True
        except Exception as e:  # noqa: BLE001
            rospy.logerr('重连失败: %s', e)
            return False

    # ---------- 主循环 ----------
    def spin(self):
        n_err = 0
        while not rospy.is_shutdown() and not self._stop.is_set():
            try:
                if self.master is None and not self.reconnect('初始连接'):
                    time.sleep(2.0)
                    continue

                types = ['HIGHRES_IMU']
                if self.uplink is not None:
                    # mode=0 时 extra_types() 返回空 —— 行为与加视觉之前完全一致
                    types = types + self.uplink.extra_types()

                msg = self.master.recv_match(type=types, blocking=True, timeout=1.0)
                if msg is None:
                    if self.uplink is not None:
                        # 超时也转一圈：让视觉模块能热切换模式、判参考过期
                        self.uplink.tick(self.master, self.vision_ref, time.time())
                    continue

                if msg.get_type() == 'HIGHRES_IMU':
                    self.publish(msg)
                elif self.uplink is not None:
                    self.uplink.on_mavlink(msg)      # 遥测镜像（只存+发ROS，不阻塞）

                if self.uplink is not None:
                    self.uplink.tick(self.master, self.vision_ref, time.time())
                n_err = 0

            except Exception as e:  # noqa: BLE001
                n_err += 1
                rospy.logwarn('读取异常 (%d): %s', n_err, e)
                if n_err >= 15:
                    if self.reconnect('连续异常'):
                        n_err = 0
                    else:
                        time.sleep(0.5)
                else:
                    time.sleep(0.05)

        if self.master is not None:
            try:
                self.master.close()
            except Exception:  # noqa: BLE001
                pass

    def publish(self, msg):
        now = time.time()
        stamp, diag = self.clock.update(msg.time_usec * 1e-6, now)

        # 单调性检查：时间戳必须一行一行往上走
        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                rospy.logwarn_throttle(5.0, '时间戳未前进: %.9f <= %.9f',
                                       stamp, self.last_stamp)
                stamp = self.last_stamp + 1e-6
            elif stamp - self.last_stamp < 5e-4:
                self.n_short_dt += 1
        self.last_stamp = stamp

        m = Imu()
        m.header.stamp = rospy.Time.from_sec(stamp)
        m.header.frame_id = FRAME_ID
        m.header.seq = self.n_pub

        # PX4 body FRD -> ROS body FLU（与香橙派桥、与标定包一致）
        m.angular_velocity.x = msg.xgyro
        m.angular_velocity.y = -msg.ygyro
        m.angular_velocity.z = -msg.zgyro

        m.linear_acceleration.x = msg.xacc
        m.linear_acceleration.y = -msg.yacc
        m.linear_acceleration.z = -msg.zacc

        # HIGHRES_IMU 不含姿态
        m.orientation.x = 0.0
        m.orientation.y = 0.0
        m.orientation.z = 0.0
        m.orientation.w = 1.0
        m.orientation_covariance[0] = -1.0

        self.pub.publish(m)
        self.n_pub += 1

        # 留一份给周期报告：scale 是"飞控采样时钟相对于宿主时钟被拉伸了多少"。
        # 它直接乘进 IMU 的时间轴 —— 错了就是预积分 dt 系统性错，静默发散。
        self.last_scale_ppm = diag.get('scale_ppm', 0.0)
        self.last_offset = diag.get('clock_offset', 0.0)

        # 给视觉上行留的时钟参考对：这一条 IMU 的（飞控采样 µs，映射后宿主时刻）。
        # 视觉时间戳要做的是反向换算（宿主 epoch → 飞控时间基），这里是最新锚点。
        self.vision_ref = (msg.time_usec, stamp, self.last_scale_ppm)

        if self.csv is not None:
            self.csv.write('%.9f,%.9f,%.9f,%.1f\n'
                           % (msg.time_usec * 1e-6, now, stamp, self.last_scale_ppm))

        if diag.get('clock_reset'):
            rospy.logwarn('时钟基准跳变，已重新锚定（累计 %d 次）',
                          self.clock.n_clock_reset)

        self.report()

    def report(self):
        now = time.time()
        if now - self.t_last_report < 20.0:
            return
        dt = now - self.t_last_report
        rate = self.n_pub / max(1e-9, now - self.t_start)
        rospy.loginfo('已发 %d 条，均速 %.1fHz；近 %.0fs %.1fHz；'
                      'dt<0.5ms %d 次；时钟跳变 %d 次；scale %+.1f ppm；offset %.6f',
                      self.n_pub, rate, dt, (self.n_pub - getattr(self, '_n_last', 0)) / dt,
                      self.n_short_dt, self.clock.n_clock_reset,
                      self.last_scale_ppm, getattr(self, 'last_offset', 0.0))
        self._n_last = self.n_pub
        self.t_last_report = now


def main():
    rospy.init_node('px4_imu', anonymous=False)
    rospy.loginfo('IMU 时钟映射模式 = %s', CLOCK_MODE)
    node = Px4ImuNode()

    def _shutdown():
        node._stop.set()

    rospy.on_shutdown(_shutdown)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    rospy.loginfo('退出：共发 %d 条，时钟跳变 %d 次', node.n_pub,
                  node.clock.n_clock_reset)
    try:
        print(node.clock.summary_text())
    except Exception:  # noqa: BLE001
        pass


if __name__ == '__main__':
    main()
