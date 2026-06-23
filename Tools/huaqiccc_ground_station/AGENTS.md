# Ground Station AI Context

> 本文档供后续 Kimi Code CLI 对话快速恢复上下文。  
> 更新时间：2026-06-16

---

## 1. 当前架构（已验证可用）

```
NOKOV (192.168.1.5) Tracker1
    │
    ▼ ENU
VRPN client (roslaunch vrpn_client_ros)
    │
    ▼
topic_tools throttle 40Hz
    │
    ▼
/mavros/vision_pose/pose  (MAVROS 处理 ENU→NED + 时间戳)
    │
    ▼ MAVLink
HM30 (天空端) ──无线── HM30 (地面端 /dev/ttyUSB0)
    │
    ▼
Pixhawk 6C (FCU)
```

控制链路（地面站 → MAVROS → FCU）：
```
control_ground_station_ros.py
    │ ROS Service/Topic
    ▼
/mavros/cmd/arming, /mavros/set_mode, /mavros/param/set, /mavros/setpoint_position/local
    │
    ▼
MAVROS（独占 /dev/ttyUSB0）
```

**无串口竞争**。MAVROS 独占串口，地面站走 ROS。

---

## 2. 关键文件

| 文件 | 作用 | 状态 |
|------|------|------|
| `start_ground_station.sh` | 一键启动：VRPN + MAVROS + throttle + 地面站 | ✅ 主力 |
| `scripts/control_ground_station_ros.py` | ROS-based 控制地面站（PyQt5） | ✅ 当前在用 |
| `src/main.py` | 完整 GUI 地面站（视频+YOLO+任务） | 待集成 |

> 历史脚本已归档到 `~/Projects/backup/ground_station_deprecated_2026-06-14/`
| `src/ground_station.py` | 完整 GUI 主窗口 | 同上 |
| `src/ros_interface.py` | ROS Topic 封装 | 同上 |

---

## 3. 控制器模式映射（MPCA_MODE）

源码定义在 `PX4/SEU_MD_PX4/src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.hpp`：

| MPCA_MODE | 实际控制器 | 地面站按钮 |
|-----------|-----------|-----------|
| 0 | Original PX4 PositionControl | "PX4 PosCtl" ← 首飞保守建议 |
| 1 | Gain-Scheduled PID | "GPID" |
| 2 | LQR Gain Scheduling | "LQR" ← Airframe 默认值 |
| 3 | Linear MPC | "MPC" |

Airframe 默认：`param set-default MPCA_MODE 0`（2026-06-11 改为 0 首飞保守）

**历史错误**：曾写成 0=PID, 1=LQR, 2=MPC，已于 2026-06-10 修正。

---

## 4. 硬件配置

| 组件 | 配置 |
|------|------|
| FCU | Pixhawk 6C（刷入定制固件 v1.14.3）|
| 遥控器 | 美国手，`COM_RC_IN_MODE=3`（RC + Joystick 双模式）|
| 动捕 | NOKOV @ 192.168.1.5，tracker="Tracker1" |
| 图传/数传 | HM30，地面端 `/dev/ttyUSB0` @ 115200 |
| 摄像头 | SIYI RTSP `rtsp://192.168.144.25:8554/main.264` |
| 变形机构 | AS5600 编码器 (I2C bus 2) + AUX1 PWM 电动推杆 |

---

## 5. 关键飞控参数

```
SYS_AUTOSTART=4401          # huaqiccc 实机 airframe（4400 旧版已删除）
COM_RC_IN_MODE=3            # 3=RC+Joystick 双模式
EKF2_EV_CTRL=15             # vision 位置+速度+姿态融合
EKF2_HGT_REF=3              # 3=vision 高度
EKF2_GPS_CTRL=0             # 禁用 GPS
MPCA_MODE=0                 # 首飞保守：原始 PID（验证后切 2=LQR）
MPCA_FF_MASS=1.15           # 飞行器质量
MPC_THR_HOVER=0.45          # 悬停油门
MORPH_EN=1                  # airframe 默认启用变形（首次验证建议确认 AS5600）
CBRK_IO_SAFETY=22027        # 禁用 IO 安全开关
PWM_MAIN_MIN/MAX=1100/1900  # ESC 行程限制
```

---

## 6. 固件状态（2026-06-14）

### 当前代码库修改
| 文件 | 修改内容 |
|------|---------|
| `huaqiccc_motor_lut.hpp` | motor 顺序修正为 `0=rb,1=rf,2=lb,3=lf`；index=0 基准值更新；MAX_ANGLE=-0.40 |
| `ControlAllocator.cpp` | `km[4]={-0.06,+0.06,+0.06,-0.06}` 匹配 airframe |
| `4401_huaqiccc_real` | 全部参数按实机测量值更新；`COM_RC_IN_MODE=3`；PWM 1100-1900 |
| `4400_gazebo-classic_huaqiccc` | SITL 电机参数同步为实机值 |
| `huaqiccc.sdf` | motor plugin 编号/转向重新映射 |
| `4400_huaqiccc`（实机目录）| **已删除** — 旧版错误 SITL airframe 错误放置在实机 ROMFS |
| `MulticopterPositionControl.cpp/.hpp` | 栖停 FSM：XY 软化、Z 增益保持、取消积分重置、删除 6s 兜底、修正 setpoint override |
| `mc_pos_control_params.c` | `MPCA_PC_K_SOFT=0.50`、`MPCA_PC_SPRING=0`、新增 `MPCA_PC_ARM_THR=-0.15` |
| `control_ground_station_ros.py` | 检测到 `CONTACT/COMPLIANT` 自动下发机械臂收拢指令（0.0 rad） |

### 编译状态
- `px4_sitl_default`：✅ 通过（18/18）
- `px4_fmu-v6c_default`：✅ clean build 通过（890/890）

### 刷写状态
- **已刷写**：6月11日最新固件已成功刷入 Pixhawk 6C 并完成实机飞行验证
- **最新固件路径**：`build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`
- **实机验证内容**：
  - ✅ 正常起飞
  - ✅ 位置模式（POSCTL）/ OFFBOARD 控制
  - ✅ 变形功能在飞行中可靠工作
  - ✅ LUT 查表在变形过程中实时更新正常
  - ✅ 四种算法（PID / GS-PID / LQR / MPC）控制可靠
- ✅ 位置/姿态接触检测实机验证通过（2026-06-14）
- ✅ 接触/栖停遥测通过 `DEBUG_FLOAT_ARRAY` 到达地面站
- **刷写后验证命令**：`ver all` → `param show MORPH_EN` → `dmesg | grep huaqiccc`

---

## 7. 常见陷阱

1. **机型选择错误**：QGC 中必须选 `4401_huaqiccc_real (Real Hardware)`。旧版 `4400` 已删除，如果还能看到说明刷的是旧固件。
2. **串口竞争**：旧版 pymavlink 直连地面站（已归档）会直接打开 `/dev/ttyUSB0`，会与 MAVROS 冲突。**只能用 `control_ground_station_ros.py`**。
3. **遥控器被屏蔽**：`COM_RC_IN_MODE=1` 会导致遥控器无效，当前已改为 `3`（双模式）。
4. **EKF2 不融合**：`EKF2_EV_CTRL=15` 和 `EKF2_HGT_REF=3` 必须同时设置。
5. **Tracker 名称**：VRPN tracker 名必须与 NOKOV 软件中设置的一致（当前是 `Tracker1`）。
6. **变形模块已启用**：当前 `MORPH_EN=1`，`huaqiccc_morph_control` 正常启动，实机飞行中 LUT 已验证工作正常。
7. **固件版本不匹配**：如果 QGC 搜不到 `MORPH_EN`，说明固件不含 morph 模块，需要重新编译刷写。
8. **自主栖停收拢**：当 `MPCA_PC_EN=2` 且 PX4 进入 `CONTACT/COMPLIANT` 时，地面站会自动下发一次收拢指令。若未看到 `[PERCH] 接触确认，已自动下发机械臂收拢指令`，检查 `/mavros/debug_value/debug_float_array` 是否到达。

---

## 8. 启动命令速查

```bash
# 一键启动（当前主力）
cd ~/Projects/ground_station && ./start_ground_station.sh

# 手动检查飞控状态
rostopic echo -n 1 /mavros/state
rosservice call /mavros/param/get "param_id: 'MPCA_MODE'"
rosservice call /mavros/param/get "param_id: 'MORPH_EN'"

# 检查动捕数据
rostopic echo -n 1 /vrpn_client_node/Tracker1/pose

# 检查 vision 是否到达 MAVROS
rostopic hz /mavros/vision_pose/pose

# 编译 v6c 固件
cd ~/Projects/PX4/SEU_MD_PX4 && make px4_fmu-v6c_default

# 刷写固件（命令行，或直接用 QGC）
python3 ~/Projects/PX4/SEU_MD_PX4/Tools/px_uploader.py \
  --port /dev/ttyACM0 \
  ~/Projects/PX4/SEU_MD_PX4/build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

---

## 9. 变形控制闭环（2026-06-11 晚）

### 架构
```
地面站 31440 命令 → MAVROS → MAVLink → mavlink_receiver
                                              ↓
                                    huaqiccc_morph_cmd (uORB)
                                              ↓
                                    huaqiccc_morph_control (模块)
                                              ↓
                                    ┌─────────────────────┐
                                    │ Bang-Bang 闭环控制   │
                                    │ error = target - cur │
                                    │ error < -死区 → +1.0 │ → 2000us 伸长(展开)
                                    │ error > +死区 → -1.0 │ → 1000us 缩短(收拢)
                                    │ |error| ≤ 死区 → 0.0 │ → 1500us 停止
                                    └─────────────────────┘
                                              ↓
                                    actuator_test (绕过解锁检查)
                                              ↓
                                    pwm_out AUX1 → 电动推杆
                                              ↓
                                    AS5600 编码器 → 角度反馈
```

### 关键教训
- **推杆是 Bang-Bang 执行器**：只有 2000us(伸长)/1000us(缩短)/1500us(停止) 三种状态。比例 PWM（如 1600us）= 停止，这是之前推杆不动的根本原因。
- **比例映射 → Bang-Bang 闭环**：之前错误地把 target 角度线性映射到 [-1,1] PWM，导致 target=-0.3 时输出 1500us（停止）。现在改为编码器反馈闭环。
- **vehicle_command 多实例问题**：mavlink_receiver 的 `_cmd_pub` 发布到实例1，模块订阅实例0，命令丢失。用专用 uORB 话题 `huaqiccc_morph_cmd` 解决。
- **Standby 状态 servo 被忽略**：飞机未解锁时 pwm_out 对 servo 通道强制输出 disarmed 值。用 `actuator_test` 话题绕过解锁检查。
- **Pixhawk 6C 刷写后必须物理断电**：软件重启不能正确退出 bootloader。

### 已验证
- ✅ 专用 uORB 话题 `huaqiccc_morph_cmd` 命令链路
- ✅ `actuator_test` 绕过解锁检查驱动 AUX1
- ✅ Bang-Bang 闭环：展开 0°→-0.4 rad ✅，收拢 -0.4 rad→0° ✅
- ✅ 地面站滑块/输入框/按钮控制（最大限制 21°）
- ✅ 电池电压 `/mavros/battery` 订阅

### 修改文件
| 文件 | 修改 |
|------|------|
| `msg/HuaqicccMorphCmd.msg` | 新增专用 uORB 命令话题 |
| `mavlink_receiver.cpp` | 31440 命令改为发布 `huaqiccc_morph_cmd` |
| `HuaqicccMorphControl.hpp` | 添加 `_morph_cmd_sub`、`_actuator_test_pub`、周期性打印计数器 |
| `HuaqicccMorphControl.cpp` | 核心：比例映射 → Bang-Bang 闭环；发布 `actuator_test` |
| `control_ground_station_ros.py` | 滑块同步输入框、最大 21°、battery 订阅、statustext / DEBUG_FLOAT_ARRAY 解析 |

---

## 10. 接触/栖停遥测（2026-06-14）

### 数据流
```
mc_pos_control (PX4)
    │
    ▼ publish debug_array (name="perch")
MAVLink DEBUG_FLOAT_ARRAY
    │
    ▼ HM30 无线链路
MAVROS debug_value plugin
    │
    ▼ /mavros/debug_value/debug_float_array
control_ground_station_ros.py
```

### 数组内容
| 索引 | 含义 | 说明 |
|------|------|------|
| `data[0]` | 接触检测状态 | 0=IDLE, 1=CANDIDATE, 2=DETECTED |
| `data[1]` | 栖停阶段 | 0=NONE, 1=APPROACH, 2=CONTACT, 3=COMPLIANT, 4=RAMP_DOWN, 5=PERCHED |
| `data[2]` | `MPCA_PC_EN` | 当前模式 |
| `data[3]` | 变形臂角度 | rad，无数据为 99.0 |
| `data[4]` | 抓握确认 | 0/1 |
| `data[5]` | 栖停激活 | 0/1 |

### 关键点
- 不依赖 `STATUSTEXT`，避免 HM30 链路对状态文本的过滤/丢失
- 保留 `/mavros/statustext/recv` 作为日志补充
- 地面站提供“自主抱柱”开关按钮，飞行中实时切换 `MPCA_PC_EN`（1=DETECT，2=FULL）
- 实机验证：2026-06-14 手动栖停测试，遥测稳定，检测灵敏度合适

---

## 11. 待办事项（最新状态 2026-06-14）

- ✅ Vision 链路验证（VRPN → throttle → MAVROS → FCU）
- ✅ 控制链路验证（地面站 → ROS → MAVROS → FCU）
- ✅ 串口竞争解决（MAVROS 独占串口）
- ✅ 控制器模式映射修正（MPCA_MODE 0/1/2/3）
- ✅ LUT 表修正（motor 顺序、index=0 基准值、MAX_ANGLE）
- ✅ ControlAllocator km 修正
- ✅ SDF / SITL airframe 同步
- ✅ 实机 airframe 更新（实机测量值基准）
- ✅ 删除错误旧机型 `4400_huaqiccc`
- ✅ v6c 固件 clean build 通过
- ✅ **变形控制闭环验证（Bang-Bang + actuator_test）**
- ✅ **刷写最新固件到飞控**
- ✅ 首飞测试（位置模式 / OFFBOARD）
- ✅ 实机 OFFBOARD 轨迹跟踪测试
- ✅ 变形 + 飞行联合测试（实机机臂展开/收拢）
- ✅ 位置/姿态接触检测实机验证（2026-06-14）
- ✅ 接触/栖停遥测链路验证（DEBUG_FLOAT_ARRAY）
- ⏳ 自主接触抱住测试（`MPCA_PC_EN=2`，人工监控）
- ⏳ 完整 GUI（`src/main.py`）集成控制器切换按钮
- ⏳ 一键栖停任务自动化
- ⏳ 视觉 YAW 对齐与实机 OFFBOARD 协调
