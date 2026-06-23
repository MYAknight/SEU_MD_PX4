# Huaqiccc 变形四旋翼 — 实机迁移说明与进度验证指南

> 生成时间：2026-06-14  
> 当前代码库：`~/Projects/PX4/SEU_MD_PX4` (PX4 v1.14.3)  
> 当前实机飞控：**Pixhawk 6C (FMU-v6c)**，数传连接于 `/dev/ttyUSB0` (HM30 @ 115200)

---

## 1. 系统组成与信息流

### 1.1 总体架构

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              上位机 / 地面站                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐  │
│  │ 测试脚本      │  │ YOLO 视觉节点 │  │ 动捕/视觉定位 │  │ QGroundControl      │  │
│  │ (Python/ROS)  │  │ (YOLO4SEU_MD)│  │ (MoCap/VIO)  │  │                     │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────────┬──────────┘  │
│         │                 │                  │                    │             │
│         └─────────────────┴──────────────────┘                    │             │
│                           │                                       │             │
│                    ┌──────┴──────┐                               │             │
│                    │   MAVROS    │◄───────────────────────────────┘             │
│                    │(ROS1 Noetic)│                                               │
│                    └──────┬──────┘                                               │
└───────────────────────────┼─────────────────────────────────────────────────────┘
                            │ MAVLink (USB / 数传)
                            ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              Pixhawk 飞控 (FMU-v6c)                             │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                        PX4 固件 (SEU_MD_PX4)                            │    │
│  │  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐   │    │
│  │  │ MulticopterPosition│───►│AdvancedPosition  │    │ ControlAllocator │   │    │
│  │  │   Control (MPC)   │    │  Control (APC)   │    │   (LUT-based)    │   │    │
│  │  └────────┬─────────┘    └────────┬─────────┘    └────────┬─────────┘   │    │
│  │           │                       │                       │             │    │
│  │           ▼                       ▼                       ▼             │    │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │    │
│  │  │              栖停 FSM (Perching State Machine)                  │   │    │
│  │  │   NONE → CONTACT → COMPLIANT → RAMP_DOWN → PERCHED              │   │    │
│  │  │   触发源：mc_pos_control 位置/姿态接触检测                        │   │    │
│  │  └─────────────────────────────────────────────────────────────────┘   │    │
│  │                                                                         │    │
│  │  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐   │    │
│  │  │ external_force_  │    │ huaqiccc_morph_  │    │   EKF2 / Sensors  │   │    │
│  │  │   estimator      │    │    control       │    │                   │   │    │
│  │  │  (IMU-based GMO) │    │ (AS5600 + AUX1)  │    │  IMU / Baro / Mag │   │    │
│  │  └──────────────────┘    └──────────────────┘    └──────────────────┘   │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                              │                                                  │
│         ┌────────────────────┼────────────────────┐                             │
│         ▼                    ▼                    ▼                             │
│     MAIN 1-4              AUX 1                I2C Bus                          │
│    (4× 电机)            (丝杆舵机)           (AS5600 编码器)                     │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 关键数据流

#### A. 飞行控制数据流
```
[MAVROS setpoint_raw/local] (ENU)
        │
        ▼
[mavlink_receiver.cpp] ──► [Trajectory Setpoint] (NED)
        │
        ▼
[MulticopterPositionControl]
        │
        ├── MPCA_MODE=0 ──► [原始 PositionControl]
        ├── MPCA_MODE=1 ──► [GS-PID]
        ├── MPCA_MODE=2 ──► [LQR]
        └── MPCA_MODE=3 ──► [MPC + FlatnessFeedforward]
        │
        ▼
[Attitude/Rates 控制器] ──► [电机混控] ──► MAIN 1-4 PWM
```

#### B. 变形机构数据流
```
[Python 脚本] ──MAV_CMD_HUAQICCC_SET_ARM_ANGLE(31440)──► [mavlink_receiver]
                                                              │
                                                              ▼
                                            统一发布 huaqiccc_morph_cmd（uORB）
                                                              │
                                                              ▼
                                            [huaqiccc_morph_control]
                                                              │
                                          ┌───────────────────┴───────────────────┐
                                          ▼                                       ▼
                                    [actuator_servos]                    [huaqiccc_morph_angle]
                                          │                                       │
                                          ▼                                       ▼
                                    AUX1 PWM (丝杆舵机)                      [ControlAllocator]
                                                                                │
                                                                                ▼
                                                                        [LUT 查表重建效率矩阵]
```

#### C. 视觉对齐数据流（新增，YOLO4SEU_MD）
```
[USB/RTSP 摄像头] ──► [FFmpeg V4L2] ──► [YOLOv8 CUDA] ──► [SORT Tracker]
                                                              │
                                                              ▼
                                                    [pixel_error = (cx - img_w/2) / (img_w/2)]
                                                              │
                                                              ▼
                              ┌───────────────────────────────┴───────────────────────────────┐
                              ▼                                                               ▼
                    [/mavros/setpoint_velocity/cmd_vel]                             [/yolo/yaw_aligned]
                              │ (twist.angular.z)                                               │
                              ▼                                                                 ▼
                    [MAVROS -> PX4 OFFBOARD]                                     [perching 脚本]
```

#### D. 栖停检测数据流

> **2026-06-14 更新**：IMU-ICD（`external_force_estimator`）在实机手动栖停测试中被证明无效——
> 原始 IMU 加速度模值在悬停时已约 25 m/s²，接触前后没有可区分的变化。该模块已从实机
> v6c board config 中移除，不再默认启动，仅作为失败尝试保留代码。
> 当前接触检测完全在 `mc_pos_control` 内实现。

```
[Local Position] ──┐
[Setpoint      ] ──┼──► [mc_pos_control Contact Detector]
[Attitude      ] ──┘         (position error + low forward vel + pitch)
                                     │
                                     ▼
                         [Perching FSM: CONTACT → COMPLIANT → ...]
                                     │
                                     ▼
                         [setpoint override + thrust ramp-down + morph]
```

检测判据（基于 2026-06-13 手动栖停日志）：
- 沿 setpoint → current_position 水平方向的位置误差 > `MPCA_PC_SERR` (默认 0.05 m)
- 该方向速度 < `MPCA_PC_SVEL` (默认 0.10 m/s)
- pitch 前倾 < `MPCA_PC_PIT_THR` (默认 -5°)
- 以上三项连续满足 > `MPCA_PC_DUR_THR` (默认 0.30 s)

---

## 2. 当前状态快照

### 2.1 仿真端（SITL）— 已完成 ✅

| 模块 | 状态 | 备注 |
|------|------|------|
| Gazebo 模型 + 机臂插件 | ✅ | `libarm_rotation_plugin.so` 已修复并验证 |
| 3 种控制器 (PID/GS-PID/LQR/MPC) | ✅ | LQR XY RMSE ~0.074m 最佳 |
| Flatness Feedforward | ✅ | 加速度前馈链路已修复 |
| LUT 控制分配 | ✅ | 31440 命令触发，矩阵实时重建 |
| 位置/姿态接触检测 | ✅ | 替换旧 IMU-ICD 与 NED-Y stall 检测，基于 2026-06-13 实机日志 |
| IMU-ICD (`external_force_estimator`) | ❌ | 实机无效，已移除默认编译/启动，代码保留 |
| Perching FSM | ✅ | CONTACT → COMPLIANT → RAMP_DOWN → MOTOR_OFF 完整 |
| 视觉 YOLO 节点 | ✅ | 29 fps @ CUDA，RTSP/USB 双模式 |

### 2.2 实机端 — V6C 已验证 ✅

| 模块 | 状态 | 备注 |
|------|------|------|
| V6C 固件编译 | ✅ | 已生成 `px4_fmu-v6c_default.px4`（FLASH 77.87%） |
| V6C 固件刷写 | ✅ | 已刷入 Pixhawk 6C，物理断电重启正常 |
| 4401 实机 airframe | ✅ | 含 EKF2 Vision、正常着陆检测参数、Morph 参数；perching 由 mc_pos_control 内位置/姿态检测触发 |
| huaqiccc_morph_control | ✅ | Bang-Bang 闭环（AS5600 反馈 + AUX1 PWM） |
| AS5600 I2C 编码器 | ✅ | 已验证，编码器行程已标定（`MORPH_EMIN=518`，`MORPH_EMAX=780`；机械实际展开极限约 raw=775） |
| AUX1 PWM 舵机输出 | ✅ | 已验证，PWM 方向与行程正确 |
| 位置/姿态接触检测 | ✅ | 2026-06-14 实机手动栖停验证，灵敏度合适 |
| 遥测链路 | ✅ | 通过 HM30 接收 `/mavros/debug_value/debug_float_array`（`name="perch"`） |
| 视觉节点实机适配 | ⚠️ | USB 摄像头已通，与实机 OFFBOARD 联调未做 |
| 动捕/VIO 定位 | ✅ | NOKOV @ 192.168.1.5 → VRPN → MAVROS 已联调 |

> 历史：2026-06-07 曾使用 Pixhawk 4 (FMU-v5) 进行早期测试，当前主力目标已切换为 FMU-v6c。

---

## 3. 实机迁移待办清单（按优先级）

### 🔴 P0 — 阻塞项（必须先做）

#### 3.1 为 FMU-v6c 编译并刷写固件 ✅ 已完成（2026-06-14）
- **状态**：V6C 是当前实机目标飞控，FLASH 77.87%，空间充足。
- **操作**：
  ```bash
  cd ~/Projects/PX4/SEU_MD_PX4
  make px4_fmu-v6c_default -j$(nproc)
  # QGroundControl 刷写：Vehicle Setup → Firmware → 自定义固件
  # 选择 build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
  # 刷写完成后必须物理断电重启
  ```
- **验证**：刷写后通过 MAVROS / QGC 读取参数，确认：
  ```
  SYS_AUTOSTART = 4401
  MPCA_MODE     = 0/2（按测试需求）
  MORPH_EN      = 1
  MPCA_PC_EN    = 1（DETECT 只记录）/ 2（FULL 触发 FSM）
  CBRK_IO_SAFETY = 22027
  ```
- **注意**：Pixhawk 6C 刷写后必须物理断电重启，软件重启不能正确退出 bootloader。

### 🟡 P1 — 重要但可并行

#### 3.5 V5 vs V6C 硬件接口差异

| 项目 | Pixhawk 4 (FMU-v5) | Pixhawk 6C (FMU-v6c) | 需确认 |
|------|--------------------|----------------------|--------|
| I2C 外部接口 | I2C A (`/dev/i2c-2`) | I2C 1/4 | AS5600 接哪个 port |
| AUX PWM | AUX1-6 (FMU PWM) | AUX1-8 | 舵机接 AUX1 |
| MAIN PWM | MAIN1-8 (PX4IO) | MAIN1-8 | 电机接线已验证 |
| 电源模块分压 | BAT1_V_DIV=18.1 | 待确认 | 是否匹配实际 PM |
| 安全开关 | 有（需 CBRK_IO_SAFETY） | 有 | 实机飞行保持禁用 |
| SD 卡 | 有 | 有 | 用于 ulog |

- **MORPH_BUS 参数**：V5 默认 I2C A 是 bus 2；如果 AS5600 接到其他 I2C 口，需改参数。

#### 3.6 动捕/定位系统集成
- 当前 `4401_huaqiccc_real` 配置为 **EKF2_EV_CTRL=15**（Vision 定位 + 高度 + 速度 + Yaw）。
- 如果使用 OptiTrack / Vicon：
  - 需要 `vrpn_client_ros` 或 `mocap_optitrack` 节点发布 `/mavros/vision_pose/pose`
  - 确认坐标系：MoCap 通常是 Z-up，需正确转换到 ENU
- 如果室外使用 GPS：
  - 需改 `EKF2_HGT_REF=1 (GNSS)`，`EKF2_GPS_CTRL=7`
  - 当前配置禁用了磁罗盘 (`EKF2_MAG_TYPE=5`)，室外可能需要恢复

#### 3.7 视觉 YOLO 与实机 OFFBOARD 联调
- 当前 `rtsp_pillar_node.py` 发布 `/mavros/setpoint_velocity/cmd_vel` 的 `twist.angular.z`。
- **实机风险**：
  - OFFBOARD 模式下只发角速度，如果其他轴没有 setpoint，PX4 可能超时退出 OFFBOARD
  - 需要保证在视觉对齐阶段，同时有位置/速度 setpoint 在发送
- **建议**：修改 `rtsp_pillar_node.py` 或 `vision_perching_integrator.py`，在发布 YAW 角速度的同时，保持当前位置作为 position setpoint。

### 🟢 P2 — 优化项

#### 3.8 电机饱和检测（Algo D）
- 论文大纲提到电机饱和检测假阳性率为 0%，当前未集成。
- 需要读取 `actuator_outputs` uORB 到 `mc_pos_control`。

#### 3.9 重新起飞（Deperch）
- 当前 Perching FSM 到 `MOTOR_OFF` 后，没有自动 Deperch 逻辑。
- 需要外部指令触发：展开机臂 → 提升推力 → 后退脱离。

#### 3.10 实机 ulog 日志分析链路
- 当前 `SDLOG_MODE=1` 从 boot 开始记录。
- 需确认 SD 卡可写，并准备分析脚本（`pyulog`）。

---

## 4. 已知风险与安全事项

### 4.1 高危参数

| 参数 | 当前值 | 风险 | 建议 |
|------|--------|------|------|
| `CBRK_IO_SAFETY` | 22027 | **禁用安全开关**，电机可在无物理确认情况下启动 | 实机飞行保持禁用（无物理安全开关） |
| `COM_DISARM_LAND` | 2.0 (PX4 default) | 着陆检测后自动 disarm | 正常降落使用；perching 由外部接触检测 + 强制 disarm 处理 |
| `LNDMC_*` | PX4 defaults | 正常着陆检测 | perching 不依赖放宽着陆检测，改为 contact_state 触发 |
| `MPCA_PC_EN` | 1 | 栖停检测默认只记录不动作 | 0=OFF, 1=DETECT（只日志）, 2=FULL（触发 FSM） |
| `MPCA_MODE` | 2 (LQR) | 实机首飞建议保守 | 首飞建议先用 Mode=0 原始 PID |

### 4.2 机械安全
- **变形机构自锁性**：丝杆在断电后应保持位置，但需要验证。
- **电机饱和**：变形展开后前后电机力臂不均，部分电机容易饱和。
- **接触冲击力**：实机撞击柱子时，Gazebo 中的理想接触不存在，需要柔性缓冲结构。

---

## 5. 新对话快速测试方法

在新对话开始时，按以下顺序执行，5 分钟内确认当前进度：

### 5.1 环境检查（30 秒）

```bash
# 1. 确认代码库路径
ls ~/Projects/PX4/SEU_MD_PX4/src/modules/huaqiccc_morph_control/
ls ~/Projects/YOLO4SEU_MD/

# 2. 确认关键文件存在且最新
grep "MORPH_EN" ~/Projects/PX4/SEU_MD_PX4/src/modules/huaqiccc_morph_control/huaqiccc_morph_control_params.c
grep "MPCA_MODE" ~/Projects/PX4/SEU_MD_PX4/src/modules/mc_pos_control/mc_pos_control_params.c
grep "CONFIG_MODULES_HUAQICCC_MORPH_CONTROL" ~/Projects/PX4/SEU_MD_PX4/boards/px4/fmu-v6c/default.px4board
```

### 5.2 编译验证（2–3 分钟）

```bash
cd ~/Projects/PX4/SEU_MD_PX4

# SITL
make px4_sitl_default -j$(nproc)

# V6C（最终目标）
make px4_fmu-v6c_default -j$(nproc)

# V5（当前测试飞控）— 注意：首次编译可能 FLASH 超限
make px4_fmu-v5_default -j$(nproc)
```

**通过标准**：
- SITL：`[XXX/XXX] Linking CXX executable px4` 无错误
- V6C：`FLASH_USED` < 100%，推荐 < 85%
- V5：当前可能 FLASH 超限，需要先精简 board config

### 5.3 飞控连接检查（1 分钟）

```bash
# 确认 HM30 地面端被识别
ls -la /dev/ttyUSB0

# 使用 QGroundControl 或 mavlink_shell 检查
# 如果有 QGC：
# qgroundcontrol

# 启动 MAVROS 后检查心跳
rostopic echo -n 1 /mavros/state
```

### 5.4 SITL 栖停仿真回归测试（可选，~90 秒）

```bash
source ~/Projects/PX4/env_seu_md_px4.sh
cd ~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite
bash runners/11_pole_collision.sh
```

**通过标准**：
- Gazebo 正常启动
- 无人机起飞 → hover → 变形 → 接近 → CONTACT_DETECTED → CONTACT → COMPLIANT
- CSV 日志生成在 `~/huaqiccc_logs/`

### 5.5 视觉 YOLO 回归测试（30 秒）

```bash
cd ~/Projects/YOLO4SEU_MD
source /opt/ros/noetic/setup.bash
python3 test_local_camera.py --device /dev/video2 --duration 5
```

**通过标准**：
- 摄像头打开成功
- FPS > 25
- 如画面中有柱状物体，应出现检测框

### 5.6 ROS 视觉节点验证

```bash
# 终端 1：启动 roscore
source /opt/ros/noetic/setup.bash
roscore

# 终端 2：启动视觉节点
source ~/Projects/PX4/env_seu_md_px4.sh
cd ~/Projects/YOLO4SEU_MD
roslaunch YOLO4SEU_MD rtsp_pillar.launch rtsp_url:=/dev/video2

# 终端 3：检查 topic
rostopic hz /yolo/pixel_error
rostopic echo /yolo/yaw_aligned
```

---

## 6. 文件速查表

| 用途 | 路径 |
|------|------|
| 项目根目录 | `~/Projects/PX4/SEU_MD_PX4` |
| 环境脚本 | `~/Projects/PX4/env_seu_md_px4.sh` |
| 实机 airframe | `ROMFS/px4fmu_common/init.d/airframes/4401_huaqiccc_real` |
| SITL airframe | `ROMFS/px4fmu_common/init.d-posix/airframes/4400_gazebo-classic_huaqiccc` |
| 变形控制模块 | `src/modules/huaqiccc_morph_control/` |
| 高级位置控制 | `src/modules/mc_pos_control/AdvancedPositionControl/` |
| 外力/接触估计器（已弃用） | `src/modules/external_force_estimator/` |
| 控制分配 LUT | `src/modules/control_allocator/huaqiccc_motor_lut.hpp` |
| 栖停仿真世界 | `Tools/huaqiccc_simulation/worlds/perching_pole.world` |
| SITL 测试脚本 | `Tools/huaqiccc_test_suite/perching/pole_collision.py` |
| SITL 运行脚本 | `Tools/huaqiccc_test_suite/runners/11_pole_collision.sh` |
| YOLO 项目 | `~/Projects/YOLO4SEU_MD` |
| YOLO 测试脚本 | `~/Projects/YOLO4SEU_MD/test_local_camera.py` |
| YOLO ROS 节点 | `~/Projects/YOLO4SEU_MD/rtsp_pillar_node.py` |
| V6C 固件产物 | `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4` |
| V5 固件产物 | `build/px4_fmu-v5_default/px4_fmu-v5_default.px4`（历史测试，当前主力为 V6C） |

---

## 7. 本次会话新增修改

- `env_seu_md_px4.sh`：添加 NVIDIA PRIME offload 环境变量
- `Tools/huaqiccc_simulation/worlds/perching_pole*.world`：关闭实时阴影，降低 GPU 负载
- `YOLO4SEU_MD/utils/rtsp_capture.py`：支持本地 USB 摄像头（FFmpeg V4L2 + MJPEG）
- `YOLO4SEU_MD/pillar_detector/rtsp_detector.py`：YOLO 模型自动加载到 CUDA
- `YOLO4SEU_MD/pillar_detector/__init__.py`：条件导入，避免 pyrealsense2 缺失崩溃
- `YOLO4SEU_MD/launch/rtsp_pillar.launch`：默认使用 `/dev/video2`，rate_hz=30
- `YOLO4SEU_MD/test_local_camera.py`：新增本地摄像头 + YOLO 快速测试脚本
- 安装 Python 依赖：`ultralytics`, `torchvision`, `filterpy`, `lap`, `scikit-image`
- 系统级：`sudo prime-select nvidia`（需注销重新登录生效）
- 编译并刷写 V5 固件到 Pixhawk 4，参数验证通过 (`SYS_AUTOSTART=4401, MPCA_MODE=2, MORPH_EN=1`)

---

## 8. 下一步建议（推荐顺序）

1. ✅ **室内动捕联调：确认 EKF2 Vision 定位稳定**（已完成）
2. ✅ **首次系留飞行：Mode=0 PID，小幅度悬停**（已完成）
3. **自主接触抱住测试**：在监控下启用 `MPCA_PC_EN=2`，验证程序自动进入 CONTACT → COMPLIANT → GRASP → RAMP_DOWN
4. **视觉 YOLO + 实机 OFFBOARD YAW 对齐测试**（重点解决 cmd_vel 与 position setpoint 协调）
5. **逐步启用 Mode=2 LQR / Mode=3 MPC**
6. **一键栖停任务自动化**：将自主接近、YAW 对齐、接触检测、变形收拢集成到地面站或 `flight_executor`

---

*本文档用于在新对话中快速恢复上下文。每次有重大进展后应更新此文档。*
