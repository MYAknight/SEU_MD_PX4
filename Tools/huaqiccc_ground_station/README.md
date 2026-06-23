# Huaqiccc 变形无人机地面站

> 专为实机迁移设计的统一地面站，集成 VRPN 动捕、MAVROS 飞控、YOLO 视觉检测于一屏。
>
> **USER_CONFIG / 路径迁移提示**
>
> 本地面站原位于 `~/Projects/ground_station`，现已并入 `SEU_MD_PX4/Tools/huaqiccc_ground_station/`。
> 一键启动脚本中仍保留原开发环境的绝对路径（如 `~/Projects/...`）。首次部署到新机器时，
> 请搜索 `USER_CONFIG` 并按实际路径修改。详见 `Tools/REALFLIGHT_PATH_MIGRATION.md`。

---

## 目录结构

```
ground_station/
├── README.md                          # 本文档
├── AGENTS.md                          # AI 快速上下文（给后续对话用）
├── start_ground_station.sh            # ✅ 一键启动脚本（当前主力）
├── config/
│   └── ground_station.yaml            # 完整 GUI 地面站配置文件（当前未使用）
├── scripts/
│   └── control_ground_station_ros.py  # ✅ 当前控制地面站（ROS+MAVROS）
└── src/                               # 完整 GUI 地面站（视频+YOLO+任务，待集成）
    ├── main.py                        # 入口
    ├── ground_station.py              # 主窗口 (PyQt5)
    ├── config_manager.py              # YAML 配置读写
    ├── ros_interface.py               # ROS Topic 封装
    └── video_widget.py                # 视频显示 + 鼠标选中

> 历史脚本（pymavlink 直连、带宽测试、旧地面站等）已归档到：
> `~/Projects/backup/ground_station_deprecated_2026-06-14/`
```

---

## 快速启动

### 推荐：一键启动（当前主力）

```bash
cd ~/Projects/ground_station
./start_ground_station.sh
```

流程：
1. 启动 VRPN client（NOKOV @ 192.168.1.5）
2. 启动 MAVROS（独占 `/dev/ttyUSB0` @ 115200）
3. 启动 topic_tools throttle（40Hz vision）
4. 等待 FCU 心跳确认
5. 启动 `control_ground_station_ros.py`（ROS-based 控制地面站）

### 完整 GUI（含视频 + YOLO）

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash  # 或 devel_isolated
export ROS_PACKAGE_PATH="~/Projects/YOLO4SEU_MD:$ROS_PACKAGE_PATH"
cd ~/Projects/ground_station
python3 src/main.py
```

然后在界面里依次点击：**初始化 ROS → 启动 VRPN → 启动 MAVROS → 启动 YOLO**

---

## 当前控制地面站说明（control_ground_station_ros.py）

### 架构

```
┌─────────────────────────────┐
│ control_ground_station_ros  │  ← PyQt5 GUI，通过 ROS Service/Topic 通信
│                             │     不直接打开串口！
└─────────────┬───────────────┘
              │ /mavros/cmd/arming
              │ /mavros/set_mode
              │ /mavros/param/set
              │ /mavros/setpoint_position/local
              ▼
┌─────────────────────────────┐
│          MAVROS             │  ← 独占 /dev/ttyUSB0（HM30 @ 115200）
│    ┌──────────────────┐     │
    │ vision_pose/pose │     │  ← 接收动捕数据（topic_tools throttle 40Hz）
    └──────────────────┘     │
└─────────────┬───────────────┘
              │ MAVLink over HM30
              ▼
┌─────────────────────────────┐
│      Pixhawk 6C (FCU)       │
└─────────────────────────────┘
```

**核心特点：无串口竞争**。MAVROS 独占串口，地面站通过 ROS 与其通信。

### 功能面板

#### 1. 状态
- 飞控连接状态（通过 `/mavros/state`）
- 飞行模式
- ARM/DISARM 状态
- 电池电压（如可用）

#### 2. 位置
- 实时位置 X/Y/Z（通过 `/mavros/local_position/pose`）
- Yaw 角度

#### 3. 目标位置
- 手动输入目标坐标
- 一键前往目标（自动切 OFFBOARD）

#### 4. 控制
- **ARM / DISARM**
- **位置模式 (POSCTL)** / **手动模式 (MANUAL)** / **OFFBOARD**
- **一键起飞 0.8m**（发送 setpoint 后切 OFFBOARD，高度由 `PERCHING_HOVER_Z` 控制）
- **降落 (LAND)**
- **控制器切换按钮**（修正后的映射）：

| 按钮 | MPCA_MODE | 实际控制器 |
|------|-----------|-----------|
| **PX4 PosCtl** | 0 | Original PX4 PositionControl |
| **GPID** | 1 | Gain-Scheduled PID |
| **LQR** | 2 | LQR Gain Scheduling（Airframe 默认值） |
| **MPC** | 3 | Linear MPC |

- **急停 (DISARM)**

#### 5. 接触 / 栖停状态
- 实时显示接触检测状态（`NO_CONTACT` / `CANDIDATE` / `CONTACT_DETECTED` / `CONTACT` / `COMPLIANT` / `GRASP_SECURE` / `RAMP_DOWN` / `PERCHED` / `ABORT`）
- 显示栖停阶段、变形臂角度、`MPCA_PC_EN`、抓握确认标志
- **“自主抱柱”开关按钮**：飞行中实时切换 `MPCA_PC_EN`
  - 关闭 = `1`（DETECT：只记录接触，不触发 FSM）
  - 开启 = `2`（FULL：程序自主进入栖停 FSM）
  - 切换后立即生效，下次启动仍保持最后设置
- **自动收拢机械臂**：当 `MPCA_PC_EN=2` 且 PX4 进入 `CONTACT` 或 `COMPLIANT` 阶段时，地面站会自动下发一次 `MAV_CMD_HUAQICCC_SET_ARM_ANGLE(0.0 rad)` 收拢指令
- 数据源：`/mavros/debug_value/debug_float_array`（`name="perch"`）
- 保留 `/mavros/statustext/recv` 作为日志补充

#### 6. 方向控制（Offboard 模式下）
- 前/后/左/右：±0.3m
- 上/下：±0.2m
- 旋转：±15°

---

## 控制器模式映射（重要！）

MPCA_MODE 在 `AdvancedPositionControl.hpp` 中的定义：

```cpp
/**
 * Mode 0: Original PositionControl (fallback)
 * Mode 1: Gain-Scheduled PID (interpolates gains based on arm_angle)
 * Mode 2: LQR Gain Scheduling (precomputed gains, state feedback)
 * Mode 3: Linear MPC (embedded QP solver)
 */
```

Airframe 默认：`param set-default MPCA_MODE 2`（即 LQR）

**注意**：之前地面站代码中映射错误（写成了 0=PID, 1=LQR, 2=MPC），已于 2026-06-10 修正为正确映射。

---

## 配置文件说明

所有可修改的硬件参数集中在 `config/ground_station.yaml`：

| 参数路径 | 默认值 | 说明 |
|----------|--------|------|
| `flight_controller.serial_port` | `/dev/ttyUSB0` | HM30 地面端串口 |
| `flight_controller.baudrate` | `115200` | 串口波特率 |
| `motion_capture.server_ip` | `192.168.1.5` | NOKOV 动捕主机 IP |
| `camera.rtsp_url` | `rtsp://192.168.144.25:8554/main.264` | SIYI RTSP 主码流 |
| `vision.Kp` | `1.5` | YAW 对齐 P 增益 |
| `perching.hover_z` | `2.5` | 栖停悬停高度（当前实际控制量为 `control_ground_station_auto.py` 中的 `PERCHING_HOVER_Z`） |

**界面修改的值会自动保存回 YAML 文件。**

---

## 完整 GUI 地面站功能（src/main.py）

### 1. 连接控制
- **初始化 ROS**：启动地面站 ROS 节点
- **启动 VRPN**：`roslaunch vrpn_client_ros sample.launch`
- **启动 MAVROS**：`roslaunch mavros px4.launch fcu_url:=serial:///dev/ttyUSB0:115200`
- **启动 YOLO**：`roslaunch YOLO4SEU_MD rtsp_pillar.launch`

### 2. 状态监控
- 飞控连接/解锁/模式
- 动捕数据流状态
- YAW 对齐状态、像素误差
- 锁定目标 ID
- 实时位置 (x, y, z)

### 3. 手动控制
- **机臂角度滑条**：精确设定变形角度，发送 MAVLink 31440
- **一键展开/收拢**：快捷按钮
- **YAW 自动对齐开关**：启用/禁用视觉 YAW 控制
- **OFFBOARD / ARM / DISARM**

### 4. 视频显示 + 鼠标选中
- 实时显示 YOLO 检测画面
- **鼠标点击检测框** → 锁定该目标（黄色框标注）
- 点击空白处 → 取消锁定，回退自动模式
- 画面中心十字线辅助对齐判断

### 5. 紧急停止
- 大红色按钮，一键：
  1. 切换飞控到 `STABILIZED`
  2. 强制 `DISARM`
  3. 终止所有外部进程

---

## 关键 Topic

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/mavros/state` | `State` | MAVROS → GS | 飞控连接/模式/ARM 状态 |
| `/mavros/local_position/pose` | `PoseStamped` | MAVROS → GS | 本地位置（NED） |
| `/mavros/vision_pose/pose` | `PoseStamped` | throttle → MAVROS | 动捕 vision 数据（ENU） |
| `/mavros/setpoint_position/local` | `PoseStamped` | GS → MAVROS | 位置 setpoint |
| `/mavros/cmd/arming` | `CommandBool` | GS → MAVROS | ARM/DISARM |
| `/mavros/set_mode` | `SetMode` | GS → MAVROS | 模式切换 |
| `/mavros/param/set` | `ParamSet` | GS → MAVROS | 参数设置（如 MPCA_MODE） |
| `/mavros/debug_value/debug_float_array` | `DebugValue` | MAVROS → GS | 接触/栖停/变形臂角度遥测 |
| `/mavros/battery` | `BatteryState` | MAVROS → GS | 电池电压 |
| `/yolo/lock_target` | `Int32` | GS → YOLO | 锁定目标 ID，-1=自动 |
| `/yolo/detections_info` | `String` (JSON) | YOLO → GS | 检测目标列表 |
| `/yolo/yaw_aligned` | `Bool` | YOLO → GS | 对齐完成标志 |
| `/yolo/pixel_error` | `Float32` | YOLO → GS | 水平像素偏差 |
| `/yolo/detection_image` | `Image` | YOLO → GS | 可视化图像 |

---

## 使用流程（实机）

```
1. 上电 → 确认 HM30 连接、摄像头网络通、动捕就绪
2. 启动地面站：./start_ground_station.sh
3. [状态监控] 确认：飞控已连接、动捕有数据
4. 无人机解锁起飞，悬停（使用遥控器或地面站 ARM + POSCTL）
5. 如需 OFFBOARD 控制：点击 OFFBOARD 按钮，然后使用方向控制
6. 如需切换控制器：点击 PX4 PosCtl / GPID / LQR / MPC 按钮
7. 任务结束：点击 降落 或 急停
```

---

## 已知问题与注意事项

### 1. 串口竞争（已解决）
- **问题**：旧版 `control_ground_station.py` 直接打开 `/dev/ttyUSB0`，与 MAVROS 竞争
- **解决**：当前主力 `control_ground_station_ros.py` 通过 ROS 与 MAVROS 通信，不碰串口
- **注意**：请勿同时运行旧版地面站和 MAVROS

### 2. 参数读写超时
- 部分参数（如 `COM_RC_IN_MODE`）通过 MAVLink 设置后读取可能超时
- 如遇到此问题，可通过 QGroundControl 手动确认参数值

### 3. EKF2 融合关键参数
以下参数必须正确设置，否则动捕数据无法融合：
- `EKF2_EV_CTRL=15`
- `EKF2_HGT_REF=3`（vision 高度）
- `EKF2_GPS_CTRL=0`（禁用 GPS）

### 4. 遥控器解锁
- `COM_RC_IN_MODE=0`（Joystick 模式会屏蔽遥控器）
- `MAN_ARM_GESTURE=1` 或正确配置 `RC_MAP_ARM_SW`

### 5. Vision 动捕 throttle 频率不匹配（临时 workaround）

**问题**：`topic_tools throttle messages` 转发 `/vrpn_client_node/Tracker1/pose` 到 `/mavros/vision_pose/pose` 时，设置频率与实际输出频率不一致。

**实测现象**（环境相关）：
- 设置 30 Hz → 实际约 20 Hz
- 设置 40 Hz → 实际约 30 Hz
- 设置 50 Hz → 实际约 38 Hz
- 无限制转发 → 约 60 Hz，但 HM30 链路严重超载（RTT > 350ms，命令超时）

**原因推断**：
- `topic_tools throttle messages` 内部使用严格大于 `>` 判定（`now - last > 1/R`），在 60Hz 输入下会导致输出“向下取整”到 60Hz 的约数。
- 实际运行时还受 HM30 链路负载、VRPN 时间抖动、多个 throttle 节点冲突等因素影响，导致读数偏离理论值。
- 替代方案 `topic_tools drop` 在当前环境下未能正常发出 MAVROS 消息，原因待查。

**当前 workaround**：
在 `start_ground_station.sh` 中将 throttle 设置频率设为 **40 Hz**，实际可获得约 **30 Hz** 的稳定动捕数据输出，满足飞控要求。

```bash
rosrun topic_tools throttle messages /vrpn_client_node/Tracker1/pose 40.0 /mavros/vision_pose/pose
```

**详细分析见**：`../待解决bug/topic_tools-vision-throttle-frequency-mismatch.md`

---

## 待完善项

- [x] 一键栖停任务自动化（phase 1-2 起飞+对齐已完成，phase 3-7 待实机验证）
- [ ] 完整 GUI 地面站（`src/main.py`）中集成控制器切换按钮
- [x] 接触/栖停状态可视化（通过 `/mavros/debug_value/debug_float_array`）
- [ ] 飞行数据记录（CSV / ulog 下载）
- [ ] 多目标跟踪列表（右侧显示所有检测目标 ID，支持列表点击选中）

---

## 故障排查：HM30 OFFBOARD 起飞后视频中断 / 自动切回 POSCTL

### 现象
- 起飞前 RTSP 视频（`rtsp://192.168.144.25:8554/main.264`）正常。
- 进入 `OFFBOARD` 并 ARM 起飞后，YOLO 画面短暂停止刷新（约 10 秒后自动恢复）。
- 任务曾因 `等待位姿稳定超时` 自动切回 `POSCTL`。

### 根因分析（已验证）
实机测试证明，主要根因**不是 HM30 带宽不足**，而是：

1. **起飞瞬间机载电压跌落**
   - 日志记录到电池从 **12.51V 跌到 11.82V**（4S 电池约 2.95V/节）。
   - 大电流导致 HM30 天空端 / SIYI 相机供电不稳，相机重启，RTSP 中断。
   - 电压稳定后相机自动恢复，视频流恢复。

2. **起飞稳定判据过严**
   - 原来的判据是 `dz < 0.1m` + `v < 0.05m/s`，且 `/mavros/local_position/pose` 只有约 **2.6Hz**。
   - 电压跌落导致飞机抖动 + 低速率位置数据差分出的速度噪声大，20 秒内无法判定为“稳定”。
   - `_wait_stable()` 超时后进入 `ABORT`，`_cleanup()` 主动切 `POSCTL`。

> 注：HM30 官方手册说明它是 RC + datalink + video 一体化链路，并支持 230400 波特率；但在本项目中带宽瓶颈未被证实为直接原因。

### 已做的软件侧优化
1. **视频丢失时悬停等待恢复**
   - `PerchingMission._wait_video_recovery()`：ALIGN / APPROACH 阶段如果 `/yolo/node_alive` 丢失，先悬停并冻结阶段超时，等视频恢复后继续任务，不再直接 abort。
   - 参数：`video_recovery_timeout = 60.0`（秒）。

2. **放宽起飞稳定判据**
   - `PERCHING_HOVER_Z` 从 `1.5m` 降到 **`0.8m`**，减少起飞电流冲击。
   - 稳定阈值：`dz < 0.25m`，`v < 0.15m/s`。
   - 速度计算改用**低通滤波**，以容忍 2.6Hz 的离散位置数据。
   - `_wait_stable()` 超时从 20s 延长到 **45s**，并需要**连续 1.0s 稳定**才进入下一阶段。

3. **遥测诊断**
   - `control_ground_station_auto.py` 每 5 秒打印 topic 实际频率。
   - 起飞前仍通过 `MAV_CMD_SET_MESSAGE_INTERVAL` 把 `LOCAL_POSITION_NED`、`ATTITUDE`、`ATTITUDE_QUATERNION` 限制到 10 Hz。

4. **波特率支持**
   - `start_ground_station_auto.sh` 支持环境变量 `HM30_BAUD`：
     ```bash
     HM30_BAUD=230400 ./start_ground_station_auto.sh
     ```

### 当前状态
- **phase 1-2（起飞 + YAW 对齐）已可完整执行**。
- 起飞后视频仍会短暂中断，但十几秒内自动恢复，任务继续。
- 若后续 phase 3+（展开、接近、盲推）需要更高遥测速率，仍可考虑把 HM30 波特率提到 230400。

### 仍建议检查的硬件问题
- **电池**：起飞压降 1V 说明电池内阻较大或电量不足，建议换满电电池测试。
- **供电分离**：给 HM30 天空端 / SIYI 相机加独立 BEC 或稳压电容，避免和电机共用电池大电流路径。
- **位姿频率**：如果 `/mavros/local_position/pose` 长期只有 2–3Hz，建议用 QGroundControl 提高 `MAV_X_RATE` 并把 `SER_TELx_BAUD` 提到 230400。

### 如果仍切回 POSCTL
查看日志：
- 如果是 `等待位姿稳定超时` → 继续放宽阈值或进一步降低 `PERCHING_HOVER_Z`。
- 如果是 `YAW 对齐超时` → 视频恢复时间超过 `video_recovery_timeout`，需等硬件稳定后再试。
- 如果是 `飞控退出 OFFBOARD -> POSCTL` 且无 abort 日志 → PX4 自身 OFFBOARD 丢失保护触发，检查 setpoint 是否停止发送。

