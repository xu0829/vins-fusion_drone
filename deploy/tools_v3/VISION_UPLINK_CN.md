# VINS → PX4 视觉定位上行：完整链路 / 用法 / 判据

> 香橙派位置：`~/vins_build/tools_v3/`  ≡  容器内 `/root/xu_drone/live/`（bind mount，同一份文件）
> 代码：`vision_uplink.py`（注入模块）+ `px4_imu_node.py`（宿主进程，5 处钩子）
> 备份：`px4_imu_node.py.bak-vision-20261008`、`vision_uplink.py.bak-102mode-20261007`
> 固件：PX4 v1.17.0（vendored 源码在笔记本 `~/drone/try_vins_again2/px4_src_v117/`）

---

## 0. 现状一句话（2026-10-08 实测）

**已经融合上了。** 台架静止，`mode=3`，飞控 EKF 的水平位置被视觉"接管"：

| 时刻 | EKF NED | flags | 说明 |
|------|---------|-------|------|
| 切 mode 3 之前 | (4.922, 6.710, −0.320) | `0x0ef` | 飞控自己积分的旧位置 |
| 切换后 **1 秒** | **(0.005, −0.003, −0.320)** | **`0x1ef`** | ← 跳到我们送的值 = `resetHorizontalPositionTo()` 执行了 |

- 最近 60 s：EKF 水平位置 vs 我们送去的值，**RMS 偏差 0.2 cm**（p90 0.4 cm）——持续跟随，不是一次性。
- 高度恒差 **−0.2998 m**，这是 `EKF2_HGT_REF=0`（气压）下的固定偏置，**设计如此**，不是故障。
- 链路时序：相机 **15.006 Hz**、`/imu` **196 Hz**、VINS odom **7.5 Hz**、端到端延迟 **78~133 ms 且平稳**。

---

## 1. 完整链路（从红外图到飞控状态估计）

```
 ┌────────────── 香橙派 / 容器 vins_pi ──────────────────────────────────────────┐
 │                                                                              │
 │  RealSense D430 ──/camera/infra1,infra2 (15Hz, 640×480)──┐                    │
 │                                                          ▼                    │
 │  PX4 ──HIGHRES_IMU 200Hz──MAVLink──► px4_imu_node ──► /imu (196Hz) ──► VINS    │
 │         (同一条 /dev/ttyACM0)          │  ▲              (宿主时钟, ENU/FLU)   │
 │                                        │  │                                   │
 │                                        │  └── /vins_estimator/odometry (7.5Hz) │
 │                                        ▼                                      │
 │                            ┌────────────────────────────┐                     │
 │                            │ vision_uplink.py（模块）    │                     │
 │                            │  ① ENU/FLU → NED/FRD       │                     │
 │                            │  ② 图像时刻 → 飞控时钟      │                     │
 │                            │  ③ 组 ODOMETRY(331) 帧      │                     │
 │                            └────────────┬───────────────┘                     │
 │                                         │ 同一条串口（绝不另开 reader）        │
 └─────────────────────────────────────────┼──────────────────────────────────────┘
                                           ▼
 ┌──────────────────────── 飞控 PX4 v1.17 ──────────────────────────────────────┐
 │  mavlink_receiver.cpp:1379  MAV_FRAME_LOCAL_FRD → pose_frame = POSE_FRAME_FRD │
 │            ↓                                                                  │
 │  /fmu/in/vehicle_odometry (uORB)                                              │
 │            ↓                                                                  │
 │  EKF2  ev_pos_control.cpp:88  FRD 分支（★ 无 yaw_align 要求）                  │
 │            ↓  用 _ev_q_error_filt 把视觉系旋进 EKF 系                          │
 │  ekf_helper.cpp:703  ESTIMATOR_STATUS.flags                                    │
 │  LOCAL_POSITION_NED ──MAVLink──► 回到 px4_imu_node ──► /px4/local_position_ned │
 └───────────────────────────────────────────────────────────────────────────────┘
```

### 1.1 ★ 为什么必须发 331(ODOMETRY)，不能发 102(VISION_POSITION_ESTIMATE)

这是这次调试最贵的一课，**102 在本机结构性地融不进去**：

1. `mavlink_receiver.cpp:1321`：102 号的 `pose_frame` 被**硬编码**成 `POSE_FRAME_NED`，
   发送方没有任何字段能改它。
2. `ev_pos_control.cpp:69`：NED 分支要求 `_control_status.flags.yaw_align` 为真，
   否则 `continuing_conditions_passing = false`（:81）→ `stopEvPosFusion()` →
   **静默拒绝**（`ev_pos` 本来就是假，stop 是空操作，日志里一个字都不留）。
3. `yaw_align` 全固件只有 4 处能置位：磁力计（`mag_control.cpp:301`）、GPS
   （`gps_control.cpp:532` / `gnss_yaw_control.cpp:110`）、**视觉 yaw**
   （`ev_yaw_control.cpp:166`）、以及上电初值（`ekf.h:429`）。
   本机 `CAL_MAG0_ID=0`（无磁）、无 GPS、`EKF2_EV_CTRL=3`（**yaw 位关**）
   ⇒ `yaw_align` 永远是假 ⇒ **102 一发就会被丢，且没有任何 MAVLink 层反馈**。
   （实测：连发 1.1 万帧，EKF 位置一动不动。）

**331 号 ODOMETRY 的 `frame_id` 是发送方决定的**（`mavlink_receiver.cpp:1379`
`MAV_FRAME_LOCAL_FRD` → `POSE_FRAME_FRD`），而 FRD 分支
（`ev_pos_control.cpp:88-118`）**没有 yaw_align 要求**：`ev_yaw` 为假时它自己用
`R_ev_to_ekf = Dcmf(_ev_q_error_filt.getState())` 把量测转进 EKF 系（:99-101）。

**代价与要求：位置和四元数必须在同一个系里** ⇒ mode 3 送的是**原始视觉系**
（`ned` + `q_ned_frd`，**不补 δ**）。补了 δ 反而少转一次。

> ⚠️ 推论：如果哪天把 `EKF2_EV_CTRL` 改成含 yaw 位（3→11），`ev_yaw` 会变真，
> FRD 分支会走 `pos = ev_sample.pos`（:91，**不再旋转**）——那时该发哪种数据
> **必须先重新推导**，不能想当然。改参数是另一件事，见 §9。

### 1.2 为什么视觉上行必须挤进 `px4_imu_node`

`/dev/ttyACM0` 只有一条，tty 字节流只交给一个 reader。另起进程开同一个口，
两边都读到残缺字节（2026-09-28 用 `fc_ping.py` 实测：多读者 = 随机坏）。
所以视觉注入、飞控遥测镜像**全都搭在已有那条 MAVLink 连接上**。

同理：**想在香橙派上看飞控数据，没有第二条路**。QGC / MAVProxy / pymavlink
listener 都得独占串口，一开就把 IMU 桥挤坏。本模块把飞控遥测**镜像成 ROS 话题**，
这是唯一不抢串口的看法。

---

## 2. 四个模式（热切换，不重启）

模式写在 `~/vins_build/tools_v3/vision_uplink.mode`（宿主机直接写即可），
节点**每秒重读一次**并重新握手：

| 值 | 含义 | 会发生什么 |
|----|------|-----------|
| `0` | 关 | 什么都不做；链路行为与没装模块**完全一致** |
| `1` | dry-run | 算好位姿、写日志/CSV、请求遥测流、dump 参数，**不碰飞控** |
| `2` | 真发 102 | 发 `VISION_POSITION_ESTIMATE`（NED，**补过 δ**）——**本机无效**，仅留作对照 |
| `3` | **真发 331** | 发 `ODOMETRY`（`MAV_FRAME_LOCAL_FRD`，**原始视觉系**）——**当前用的就是这个** |

```bash
echo 3 > ~/vins_build/tools_v3/vision_uplink.mode     # 开始融合
echo 0 > ~/vins_build/tools_v3/vision_uplink.mode     # 停车（回放/调试时务必先做）
```

切换瞬间日志出现 `[vision] 模式 2(...) → 3(...)`，随后立刻重新握手
（请求遥测流 + dump 参数），**不需要重启任何进程**。

---

## 3. 日常操作（照抄即可）

### 3.1 只想发数据（最常见，一步）

```bash
echo 3 > ~/vins_build/tools_v3/vision_uplink.mode
ssh 香橙派 'sudo docker exec vins_pi tail -f /tmp/imu_node_recover.log'   # 盯状态行
```

前提是栈已经在跑（下面 3.2）。模块常驻在 `px4_imu_node` 里，**改模式文件就是全部操作**。

### 3.2 启停整个栈

```bash
# 起（前台跑，Ctrl-C 优雅停止）—— 必须带 CONFIG，否则跑的是默认配置
ssh 香橙派 'sudo docker exec -it vins_pi bash -lc \
  "cd /root/xu_drone/live && CONFIG=/root/xu_drone/live/config_x10/d430_px4_live_refnoise_x10.yaml \
   bash start_vins_pi.sh"'

# 停（全程 SIGINT，绝不用 kill -9 —— 硬杀会把 D430 搞卡，只能靠 USB 重新授权救）
ssh 香橙派 'sudo docker exec vins_pi bash /root/xu_drone/live/stop_vins_pi.sh'
```

启动器顺序：预检 → roscore → 相机 + 发射器守卫 → 关发射器 → **等飞控心跳起 IMU 桥**
→ foxglove → vins_node → 诊断转发 → 复查。

`px4_imu_node` 由 `wait_fc_then_start_imu.sh` 监督拉起（等心跳再起，因为节点自身
的重连分支有死连接 bug），**stdout/stderr 重定向到 `/tmp/imu_node_recover.log`** ——
`[vision]` 状态行就在那里（`/tmp/imu_supervisor.log` 只记"什么时候拉起的"）。

### 3.3 改过代码之后：先跑自检（不碰串口，10 秒）

```bash
ssh 香橙派 'sudo docker exec vins_pi bash -lc "source /opt/ros/noetic/setup.bash && \
  python3 /root/xu_drone/live/vision_uplink_selftest.py"'
```

它验：pymavlink 协议/API（102 的 crc_extra=158、331 的 crc_extra=91）、
**发出去的字节级布局**（331 载荷 233 字节、frame_id 落在偏移 228、`<5B` 尾块）、
坐标变换 8 个数值用例、mode-3 运行时路径、reset_counter、状态行格式。
当前状态：**82 个 [OK]，0 FAIL**。

---

## 4. 怎么看（四个角度）

### ① 状态行（最常用，1 Hz）

```bash
ssh 香橙派 'sudo docker exec vins_pi tail -f /tmp/imu_node_recover.log'
```

```
[vision] mode=3(真发 331(ODOMETRY/FRD 原始视觉系)) 发=1833(8.0Hz,254s) 延迟=126ms
  δ=-98.6° 倾角差=4.0° | 最后发/FRD原始 (0.002,-0.006,-0.020)
  | EKF NED (-0.009,-0.011,-0.319) Δ=0.299m | rpy(-3.8,-1.1,100.8)°
  | flags=0x1ef[attitude,vel_h,vel_v,pos_h_rel,pos_v_abs,pos_v_agl,const_pos,pred_pos_h_rel] 已上锁
```

- `发=N(x Hz, 已跑秒数)`：累计发帧数 / 当前速率 / 节点运行时长
- `延迟`：**这一帧**的 odom 时间戳距今多久。>500 ms 模块自动停发并标 `⚠已停发Ns`
- `δ`：VINS 系 → 飞控系 的 yaw 差（mode 1 测出来的），仅参考
- `最后发/FRD原始`：mode 3 **真正发出去**的原始视觉系位置；`最后发/NED` 是补过 δ 的，只在 mode 2 有意义
- `EKF NED` / `Δ`：飞控自己的估计 / 与发送值的差（**注意 Δ 含 z，见 §5**）
- `倾角差`：我们发的姿态 vs 飞控姿态的 roll/pitch 差；VINS 静止时会慢慢漂，不是链路错
- `flags`：`ESTIMATOR_STATUS.flags`（见 §5）；`已上锁`= disarmed

### ② ROS 话题（实时数值）

```bash
ssh 香橙派 'sudo docker exec -it vins_pi bash -lc "source /opt/ros/noetic/setup.bash; \
  rostopic echo /px4/local_position_ned"'
ssh 香橙派 'sudo docker exec -it vins_pi bash -lc "source /opt/ros/noetic/setup.bash; \
  rostopic echo /px4/estimator_status"'
```

### ③ CSV（拉回笔记本慢慢看）

容器 `/tmp/vins_pi/` 下：`vision_uplink.csv`（每帧：原始 VINS → 补 δ → 真发值 →
当时飞控 EKF 位置 + flags）、`px4_telemetry.csv`、`px4_params.csv`。
**容器 `/tmp` 没有映射到宿主机**，要用 `docker exec` 读，或 `docker cp` 出来：

```bash
ssh 香橙派 'sudo docker exec vins_pi bash -lc "cat /tmp/vins_pi/vision_uplink.csv"' > vision_uplink.csv
```

列名（26 列）：`wall, odom_stamp, latency_s, mode, fc_usec, vins_x/y/z(原始世界系),
ned_x/y/z(补δ后的真北NED), roll/pitch/yaw_deg(补δ后姿态), dyaw_deg, frd_x/y/z(★mode3真正发的),
qfrd_r/p/y_deg(原始姿态), reset_n, ekf_x/y/z, flags`

### ④ Foxglove 看三维

浏览器连 `ws://192.168.2.110:8767`（有线优先；网线拔了才用 `ws://192.168.1.215:8767`），勾 `/vins_estimator/odometry` 和
`/px4/local_position_ned`，两条轨迹一起看。

---

## 5. 怎么判断"飞控真的在用视觉"

**光看数值好看没用**，要按顺序看三件事：

### 第一步：融合启动的**铁证**——位置跳变

静止台架上切到 mode 3 的那一秒，`EKF NED` 会**从飞控自己积分的旧位置跳到我们送的值**
（本次实测 `(4.922,6.710)` → `(0.005,−0.003)`）。源码依据：无 GNSS 时
`startEvPosFusion()` 会调 `resetHorizontalPositionTo(measurement,…)`
（`ev_pos_control.cpp:225`）。**没有这一跳 = 没融合。**

### 第二步：flags 变化

`ESTIMATOR_STATUS.flags`（`ekf_helper.cpp:669-721` 生成）：

| bit | 名字 | 判据 | 本次实测 |
|-----|------|------|---------|
| 0 | attitude | `attitude_valid()` | 一直亮 |
| 3 | pos_h_rel | `isLocalHorizontalPositionValid()` | 一直亮 |
| 4 | pos_h_abs | `isGlobalHorizontalPositionValid()`（**要 GPS**） | **始终 0，别拿它当判据** |
| 7 | const_pos | `fake_pos \|\| vehicle_at_rest`（静止台架**正常会亮**） | 静止时亮 |
| **8** | **pred_pos_h_rel** | `isHorizontalAidingActive()` | **切 mode 3 后亮起来 ← 这一位才是"水平有外部助航"** |

本次：`0x0ef` → `0x1ef`（多了 bit8）。

### 第三步：跟随（定量）

稳态下 EKF 水平位置应当**贴着我们送的值**（最近 60 s：RMS 0.2 cm）。
**注意读 Δ 的正确姿势**：

- `Δ` = 三维距离，**包含 z**。本机 `EKF2_HGT_REF=0`（气压主参考）⇒ 视觉高度
  只做**带偏置的修正、不重置**（`ev_height_control.cpp:170` 的 else 分支），
  所以 z 会恒定差 ~0.30 m，`Δ` 就一直显示 ~0.30 m。**这不是故障。**
- **要看到的是水平的 Δ**：`hypot(发送x−EKFx, 发送y−EKFy)`，应该在**厘米级**。

### 第四步（需要人动手）：手持移动测试

把飞机拿在手里，沿某个方向平移 **20~30 cm**，然后对比
"`最后发/FRD原始` 的变化向量" 和 "`EKF NED` 的变化向量"：

- **位移的模长应该相等**（都 ≈ 0.2 m）；
- **方向可以不同**：EKF 系比视觉系转过一个 yaw（本机 δ≈−99°，即约 100°），
  `Rz(δ)·Δ视觉 ≈ ΔEKF` 就是对的。**方向"看着不对"不等于错**，要先按 δ 转一下。
- 顺带看 yaw：旋转飞机 ~30°，`qy` 列与飞控 yaw 应该同向变化。

---

## 6. 坐标、时间戳、消息格式（改代码前先读这段）

### 6.1 变换

```
p_ned      = (p_enu.y, p_enu.x, −p_enu.z)
q_ned_frd  = Q_ENU2NED ⊗ q_world_body ⊗ Q_FLU2FRD
             Q_ENU2NED = (x=0.7071, y=0.7071, z=0,  w=0)   # 绕 (1,1,0)/√2 转 180°
             Q_FLU2FRD = (x=1,      y=0,      z=0,  w=0)   # 绕 x 轴转 180°
```

来源：`vins_estimator/src/utility/visualization.cpp:124 pubOdometry`
（`frame_id="world"`，重力对齐、z 朝上、yaw 随初始化朝向；body=FLU）。
数值已被自检的 8 个用例覆盖。

### 6.2 消息

- **102**：PX4 读的是欧拉角（`Eulerf(vpe.roll,…)`），速度字段被忽略，协方差只取
  6 个对角项 `[0][6][11]`(位置) `[15][18][20]`(姿态)。**必须 MAVLink 2**
  （`covariance` 是扩展字段；v1 发出去 PX4 会补零 = "零方差 = 无限自信"，比不传还糟）。
  `px4_imu_node.py` 在 `import pymavlink` **之前** `os.environ.setdefault('MAVLINK20','1')`。
- **331**：载荷 **233 字节**——usec@0, xyz@8, q@20, v@36, ω@48, pose_cov@60,
  velocity_cov@144, frame_id@228, child_frame_id@229, reset_counter@230,
  estimator_type@231, quality@232；crc_extra=91。速度一律 NaN（PX4 侧
  `isAllFinite()` 直接跳过速度分支，不融合）；`quality=100`；`reset_counter`
  只在视觉世界系真跳变时 +1，让 EKF 重置而不是把跳变当量测。

### 6.3 时间戳（关键，决定 EKF 怎么看延迟）

`time_usec` **不是发送时刻，是图像采集时刻**，用最近一条 IMU 的时钟参考对
线性外推映射进飞控时钟（`vision_uplink.py:748`）：

```
fc_usec = fc_ref + (odom_stamp − host_ref) × 1e6 × (1 − scale_ppm×1e-6)
```

所以那 100 ms 管线延迟是**如实申报**的，EKF 的延迟缓冲能补偿它；
`EKF2_EV_DELAY=15.0`（ms）只是再补一点点残差。

---

## 7. 参数（只读 dump，实测值 2026-10-08）

| 参数 | 实测 | 含义 |
|------|------|------|
| `EKF2_EV_CTRL` | **3** | bit0=HPOS 水平位置 ✓ bit1=VPOS 垂直位置 ✓ bit2=VEL ✗ bit3=YAW ✗（`ekf_common.h:162`） |
| `EKF2_EV_DELAY` | **15.0** | 单位 **ms**，视觉管线残差延迟 |
| `EKF2_EV_QMIN` | 0 | 质量门限（0=不卡质量） |
| `EKF2_HGT_REF` | **0** | 0=气压 1=GPS 2=测距 3=视觉 ⇒ 视觉高度**不重置** z，只做偏置修正 |
| `CAL_MAG0_ID` | **0** | 无磁力计 ⇒ `yaw_align` 无来源（§1.1 的死结） |

**本模块绝不写参数**——它只发视觉报文 + `SET_MESSAGE_INTERVAL`（只改飞控内存里的
流频率，断电即恢复）。要改参数是另一件事，必须单独确认后再做（持久化行为，
影响下一次上电）。

---

## 8. 已知限制 / 下一步

1. **VINS 姿态慢漂**：静止时 roll/pitch 会漂（本次 130 s 漂了 ~2.8°，
   状态行里体现为 `倾角差` 涨到 4°）。位置融合不受影响，但**飞行前要评估**——
   `EKF2_EV_CTRL` 没开 yaw 位，EKF 的航向不来自视觉。
2. **odom 只有 7.5 Hz**（相机 15 Hz 的一半），延迟 ~100 ms。
   本次为了压住"延迟无限增长"把相机从 30 fps 降到 15 fps（见
   `start_vins_pi.sh` 里 `infra_fps:=15` 与注释，备份 `.bak-fps30-20261007`）。
3. **状态行的 `Δ` 混了 z**：更清楚的做法是在 mode 3 下额外打印水平 Δ
   （小改动，需要重启 `px4_imu_node` 才生效，会和下次重启一起做）。
4. 高度目前**只是带偏置融合**、没有以视觉为主参考。要真以视觉定高需要改
   `EKF2_HGT_REF=3`——**那是写参数**，另议。
5. Offboard 悬停 / `SET_POSITION_TARGET_LOCAL_NED` **不在范围内**。

---

## 9. 安全红线（没变）

- **拆桨**；只在台架上做地面验证，**不自动解锁、不自动起飞**
- 本模块**只发** `ODOMETRY`/`VISION_POSITION_ESTIMATE` 和几条**只读**请求
- **绝不写参数**
- 任何时候 `pgrep -af "[p]x4_imu_node"` **超过 1 个就别开测**（两个读串口的会互相撕字节）
- 停栈一律 SIGINT（`stop_vins_pi.sh`），**绝不用 `kill -9`**
- 飞控状态：**已上锁**（状态行里的 `已上锁`）；切模式可以随时做
