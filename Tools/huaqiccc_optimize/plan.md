# Huaqiccc 变形无人机 — 下阶段实机调试与集成计划

> 生成时间：2026-06-14  
> 适用对象：项目负责人与协作 AI  
> 本计划基于 2026-06-14 已完成的实机验证：正常起飞、POSCTL/OFFBOARD、变形功能、四种控制算法、LUT 查表、位置/姿态接触检测与遥测链路均已在实机验证通过。

---

## 总体目标

在已完成的基础飞行能力之上，完成三件事：

1. **控制参数调优**：建立一套适合实机的稳定飞行参数基线，重点补齐 EKF2/位置控制/姿态控制的关键参数。
2. **自主栖停能力**：接触检测灵敏度已实机验证 → 在监控下启用 `MPCA_PC_EN=2` 验证程序自主栖停。
3. **视觉 YAW 对齐与地面站集成**：修复 YOLO 链路问题、标定并补偿视觉延迟、把 YOLO 功能接入地面站并实现手动测试。

---

## 已生成工具（位于 `~/Projects/optimize/`）

| 文件 | 作用 |
|------|------|
| `launch_env.sh` | 一键启动环境：roscore + VRPN + MAVROS + throttle + flight_executor |
| `start_optimize_station.sh` | 一键启动环境 + GUI（推荐日常使用） |
| `auto_flight_gui.py` | 简化飞行 GUI：轨迹选择 + 一键起飞/降落/急停 + 日志下载 |
| `flight_executor.py` | 实际飞行执行器：提供 `/optimize/start_flight`, `/optimize/land`, `/optimize/emergency_stop` 服务 |
| `safe_space.yaml` | 安全空间配置：实际测量四边形角点 + home 点 + 高度/速度/裕量参数 |
| `download_logs.py` | 通过 MAVLink FTP 下载最新 ulog（不使用 QGC） |
| `analyze_flight.py` | 自动分析飞行日志，生成自然语言报告和优化建议 |
| `verify_safe_space.py` | 地面前验证：绘制安全空间与轨迹，检查最小边界距离 |
| `plan.md` | 本文档 |

### 快速开始

```bash
cd ~/Projects/optimize

# 1. 启动完整环境（roscore + VRPN + MAVROS + flight_executor）
./launch_env.sh

# 2. 在另一个终端启动简化 GUI
python3 auto_flight_gui.py

# 3. GUI 中选择轨迹和控制器，点击"一键起飞并执行轨迹"

# 4. 飞行结束后下载日志
python3 download_logs.py

# 5. 分析日志
python3 analyze_flight.py
```

---

## 一、任务 1：控制参数调优

### 1.1 目标

建立一套在实机上稳定飞行的参数基线，覆盖 EKF2 外部视觉融合、位置控制、姿态控制、以及四种 MPCA_MODE 下的控制参数。

### 1.2 输入

- 当前 airframe：`ROMFS/px4fmu_common/init.d/airframes/4401_huaqiccc_real`
- 当前 SITL 参数注入：`ROMFS/px4fmu_common/init.d-posix/px4-rc.params`
- 已有文档：`TUNING_BASELINE_PARAMS.md`、`TUNING_RESULTS_v1.md`

### 1.3 步骤

#### 步骤 1：建立参数变更追踪表

新建文件：`/home/a/Projects/PX4/SEU_MD_PX4/TUNING_FLIGHT_LOG.md`

每次飞行前记录：

```markdown
| 日期 | 版本 | 变更参数 | 旧值 | 新值 | 变更原因 | 试飞结果 |
|------|------|---------|------|------|---------|---------|
```

#### 步骤 2：标定安全飞行空间并设计标准轨迹

**当前采用实际测量安全空间（已更新到 `~/Projects/optimize/safe_space.yaml`）**：

```yaml
safe_space:
  corners:
    - [1.233, 1.542, 0.1]    # 左前
    - [-0.433, 1.337, 0.1]   # 左后
    - [-0.5, -0.246, 0.1]    # 右后
    - [1.432, -0.305, 0.1]   # 右前
  home: [0.332, 0.727, 0.1]
  z_min: 0.1
  z_max: 2.0

mission:
  flight_height: 1.0
  safety_margin: 0.30   # 轨迹/飞行器距边界 ≥ 0.30 m
```

home 点到四边形边界最短距离约 **0.699 m**（到左前边）。扣除 0.30 m 安全裕量后，轨迹可用最大半径约 **0.40 m**。当前轨迹尺寸统一控制在半径/半长 **≤ 0.30 m**，所有轨迹到边界最小距离 ≥ 0.39 m。

**验证工具**：
```bash
cd ~/Projects/optimize
python3 verify_safe_space.py
```
会输出每条轨迹的最小边界距离并保存可视化图 `safe_space_verify.png`。

标准轨迹库（实际测量安全空间，home 为中心）：

| 轨迹 | 尺寸 | 速度 | 用途 |
|------|------|------|------|
| `hover` | 中心点 | 0 | 稳态悬停、Z 轴漂移 |
| `takeoff_land` | 中心垂直 | 0.2 m/s | 起降平稳性 |
| `square_small` | 边长约 0.40 m | 0.3 m/s | XY 轴向位置跟踪 |
| `circle_small` | 半径 0.30 m | 0.3 m/s | 连续曲线跟踪 |
| `figure8_small` | 半径 0.30 m 的 8 字 | 0.3 m/s | 连续变向耦合 |
| `step_x` | ±0.22 m | 0.3 m/s | X 轴阶跃响应 |
| `step_xy` | 对角 ±0.22 m | 0.3 m/s | XY 耦合响应 |
| `morph_circle` | 半径 0.30 m 的圆 | 0.3 m/s | 变形中位置跟踪 |

脚本特性：
- 使用绝对坐标（动捕坐标系）
- 起飞前检查初始位置是否在安全空间内
- 飞行中实时监控位置，越界时自动返回中心并降落
- 支持 `MPCA_MODE 0/1/2/3` 切换控制算法
- `morph_circle` 轨迹会在 1/4 和 3/4 处自动发送变形命令
- 所有航点在发布前均经过 `SafePolygon.clip_point` 裁剪，保证不超出安全裕量

使用示例：
```bash
cd ~/Projects/optimize
./launch_env.sh
# 在另一个终端
python3 auto_flight_gui.py
```
在 GUI 中选择轨迹和控制器，点击"一键起飞并执行轨迹"。

#### 步骤 3：补齐 EKF2 关键参数

当前 airframe 已设置 `EKF2_EV_CTRL=15`、`EKF2_HGT_REF=3`、`EKF2_GPS_CTRL=0`。需要重点检查并标定以下参数：

| 参数 | 当前值 | 建议动作 | 验证方法 |
|------|--------|---------|---------|
| `EKF2_EV_DELAY` | 0.02 | 标定真实延迟（动捕→MAVROS→FCU） | 对比 `/vrpn_client_node/Tracker1/pose` 与 `/mavros/local_position/pose` 时间戳 |
| `EKF2_EV_POS_X/Y/Z` | 默认 | 测量动捕标记点与 FCU 的杠杆臂 | 直尺测量 + 飞行后分析 |
| `EKF2_EV_QMIN` | 默认 | 必要时提高/降低视觉置信度阈值 | 观察 EKF  fuse 状态 |
| `EKF2_HGT_REF` | 3 | 保持不变 | 确认无 GPS 参与高度 |
| `EKF2_MAG_TYPE` | 5 | 保持不变 | 确认航向由 vision 提供 |

**标定 `EKF2_EV_DELAY` 的标准流程：**

1. 启动地面站，确保 VRPN → throttle → MAVROS → FCU 链路正常。
2. 快速水平晃动无人机（手持或低速飞行），同时记录：
   ```bash
   rosbag record /vrpn_client_node/Tracker1/pose /mavros/local_position/pose /mavros/vision_pose/pose
   ```
3. 用 Python 计算两个位置序列的互相关延迟：
   ```python
   # 伪代码
   delay = cross_correlation(vrpn_x, local_x)
   ```
4. 将标定结果写入 airframe 的 `EKF2_EV_DELAY`。

#### 步骤 4：位置控制参数调优

当前保守基线：

```
MPC_XY_P 1.5
MPC_XY_VEL_P_ACC 2.2
MPC_XY_VEL_I_ACC 0.4
MPC_XY_VEL_D_ACC 0.2
MPC_Z_P 1.0
MPC_Z_VEL_P_ACC 4.0
```

调优流程（每次只改一个参数，幅度 ±20%）：

1. `MPCA_MODE=0`（原始 PID）下进行悬停→小范围阶跃→圆轨迹。
2. 记录 ulog，用 `px4_tools` 或 Python 分析：
   - 位置跟踪误差 std / max
   - 超调量
   - 振荡频率
3. 优先调整 `MPC_XY_P` 和 `MPC_XY_VEL_P_ACC`，再调 I/D。
4. Z 轴优先调整 `MPC_Z_P` 和 `MPC_THR_HOVER`。

#### 步骤 5：姿态控制参数调优

当前：

```
MC_ROLLRATE_P 0.18 / I 0.20 / D 0.003
MC_PITCHRATE_P 0.15 / I 0.18 / D 0.003
MC_YAWRATE_P 0.25 / I 0.30 / D 0.0
```

调优方法：

1. 在 POSCTL 下做快速俯仰/横滚杆量输入（遥控器）。
2. 观察 ulog 中 `vehicle_attitude_setpoint` 与 `vehicle_attitude` 的跟踪。
3. 若出现高频振荡，降低 P 或增加 D；若响应迟钝，增加 P。
4. Yaw 轴重点照顾 OFFBOARD 旋转和视觉 YAW 对齐时的稳定性。

#### 步骤 6：四种算法分别调优

| 模式 | 关键参数 | 调优重点 |
|------|---------|---------|
| 0 PID | `MPC_XY_*`, `MPC_Z_*` | 基准稳定 |
| 1 GS-PID | `MPCA_FF_*`, LUT 增益 | 变形过程增益平滑 |
| 2 LQR | `MPCA_FF_*`, LQR 增益表 | 默认模式，重点优化 |
| 3 MPC | `MPCA_MPC_ALPHA`, `MPCA_MPC_R_DELTA` | Z 轴稳态误差、积分器 |

每次切换模式后：

1. 悬停 30 秒，记录稳态误差。
2. 执行 2m 方框或圆轨迹，记录动态跟踪误差。
3. 在飞行中执行一次变形 0→-0.35 rad→0，观察变形期间的姿态/位置波动。

#### 步骤 7：变形飞行稳定性专项

1. 在 `MPCA_MODE=2`（默认）下，悬停并缓慢变形（地面站滑块或脚本）。
2. 记录变形期间：
   - 位置漂移（XY/Z）
   - 姿态角波动
   - 电机输出是否饱和
3. 若变形时明显漂移：
   - 检查 LUT 角度插值是否平滑
   - 调整 `MPC_FF_MASS` / `MPCA_FF_BLEND`
   - 考虑添加 CoP 偏移前馈（进阶）

### 1.4 验收标准

- [ ] `EKF2_EV_DELAY` 已标定并写入 airframe
- [ ] 四种模式下悬停位置误差 std < 10cm（XY），Z < 15cm
- [ ] 变形 0→-0.35→0 过程中位置漂移 < 30cm
- [ ] 圆轨迹（半径 2m，速度 1m/s）跟踪误差 < 20cm
- [ ] 参数变更全部记录到 `TUNING_FLIGHT_LOG.md`

### 1.5 安全要求

- 每次只改一个参数，幅度不超过 ±20%。
- 调参前确认 `MPCA_MODE=0` 作为 fallback 可用。
- 遥控器始终由操作员接管，任何异常立即切 MANUAL/DISARM。

### 1.6 自动化调优扩展（可选进阶）

在人工调优达到可接受基线后，可以引入半自动化调优：

**你提供：**
- 一组当前可稳定飞行的参数文件
- 一段标准 offboard 轨迹（可使用上面生成的 `safe_offboard_mission.py`）
- 待调优参数范围（如 `MPC_XY_P ∈ [1.0, 3.0]`）
- 安全约束和验收标准

**我生成：**
- `run_tuning_mission.py`：自动执行参数变更 + 标准飞行 + 记录元数据
- `analyze_ulog.py`：自动提取 XY/Z RMSE、超调、姿态活动度、电机饱和度等
- `suggest_next_params.py`：基于规则库推荐下一组参数

**工作流：**
```
基线飞行 → 我分析 → 生成 5 组候选参数
    ↑                          ↓
验证最优参数 ← 你依次试飞 ← 自动执行
```

**安全边界：**
- 我不能直接控制实机飞行，只能生成脚本
- 参数单次变化幅度 ≤20%
- 每次飞行前由你确认参数组合
- 所有飞行必须由操作员监控并保留遥控器接管能力

---

## 二、任务 2：自主栖停实机验证

### 2.1 目标

位置/姿态接触检测灵敏度已实机验证（2026-06-14）。本任务目标是在人工监控下启用 `MPCA_PC_EN=2`，验证程序自主完成 CONTACT → COMPLIANT → GRASP → RAMP_DOWN → PERCHED 的全流程。

### 2.2 涉及模块

- `MulticopterPositionControl` 中的位置/姿态接触检测与栖停 FSM
- `huaqiccc_morph_control`（变形收拢）
- ulog 日志记录
- 地面站 `/mavros/debug_value/debug_float_array` 遥测
- 可选分析脚本：`Tools/huaqiccc_test_suite/perching/analyze_*.py`

### 2.3 关键参数

| 参数 | 当前默认值 | 说明 |
|------|-----------|------|
| `MPCA_PC_EN` | 1 | 0=OFF, 1=DETECT（只记录）, 2=FULL（触发 FSM） |
| `MPCA_PC_SERR` | 0.05 m | 沿 setpoint 方向位置误差阈值 |
| `MPCA_PC_SVEL` | 0.10 m/s | 该方向速度阈值 |
| `MPCA_PC_PIT_THR` | -5° | pitch 前倾阈值 |
| `MPCA_PC_DUR_THR` | 0.30 s | 连续满足时间 |
| `MPCA_PC_PRELOAD` | 0.05 m | COMPLIANT 阶段弹簧预紧偏移 |
| `MPCA_PC_RAMP_T` | 2.0 s | RAMP_DOWN 推力衰减时间 |

测试前应根据 2026-06-14 实机表现确认这些阈值在手动飞行中不会误触发。

### 2.4 步骤

#### 阶段 0：确认遥测与日志

1. 启动地面站，确认 `/mavros/debug_value/debug_float_array` 能收到 `name="perch"` 消息。
2. 确认 ulog 记录以下消息：
   - `vehicle_local_position`
   - `vehicle_attitude`
   - `vehicle_local_position_setpoint`
   - `huaqiccc_morph_angle`
   - `actuator_motors` / `actuator_outputs`
   - `vehicle_control_mode`
3. 建议同步 rosbag：
   ```bash
   rosbag record /mavros/local_position/pose /mavros/setpoint_position/local \
                 /mavros/state /mavros/debug_value/debug_float_array \
                 /vrpn_client_node/Tracker1/pose
   ```

#### 阶段 1：DETECT 模式复测（`MPCA_PC_EN=1`）

- 与 2026-06-14 类似，手动完成接近-接触流程。
- 地面站观察 `data[0]` 何时从 1（CANDIDATE）变为 2（DETECTED）。
- 确认接触判断时机合适、无误触发。

#### 阶段 2：自主栖停测试（`MPCA_PC_EN=2`）

> 可通过地面站“自主抱柱”按钮在飞行中实时切换 `MPCA_PC_EN`：
> - 地面起飞→接近抱柱阶段：开启（`2`）
> - 柱上起飞→地面降落阶段：关闭（`1`）

1. 在空旷场地固定栖停杆，建议先用柔性/缓冲杆测试。
2. 手动起飞到 1.0~1.5 m（低高度，降低风险）。
3. 手动调整位置，使机头对准杆子。
4. 保持手动缓慢前推（约 0.2~0.3 m/s）或切换到 OFFBOARD 轨迹。
5. 程序检测到接触后自动：
   - 进入 CONTACT 阶段，记录接触点
   - 进入 COMPLIANT 阶段，软化位置环并维持竖直推力
   - 6 s 后若位置稳定，判定 GRASP_SECURE
   - 进入 RAMP_DOWN，指数衰减推力
   - 到达 PERCHED
6. 操作员全程手不离遥控器，异常立即切 MANUAL / DISARM。
7. 记录 ulog 和 rosbag，事后复盘。

#### 阶段 3：复盘与迭代

- 下载 ulog：`python3 download_logs.py`
- 分析：
  - 接触检测时刻与实际接触时刻是否对齐
  - COMPLIANT 阶段位置/推力是否稳定
  - GRASP 是否成功
  - 若误触发或漏检，调整 `MPCA_PC_SERR/SVEL/PIT_THR/DUR_THR`
- 必要时重新编译刷写 v6c 固件。

### 2.5 验收标准

- [x] 接触检测灵敏度已实机验证（2026-06-14）
- [ ] `MPCA_PC_EN=2` 下程序自主进入 CONTACT 阶段
- [ ] 自主进入 COMPLIANT 并维持稳定 ≥ 6 s
- [ ] 成功判定 GRASP_SECURE 并进入 RAMP_DOWN
- [ ] 最终到达 PERCHED，姿态稳定
- [ ] 全程无危险振荡或失控

### 2.6 安全要求

- 首次测试高度 ≤ 1.5 m，操作员可立即接管。
- 栖停杆必须固定牢固，接触面有缓冲。
- 建议先用柔性杆/泡沫柱测试。
- 遥控器始终由操作员持有，任何异常立即切 MANUAL 或 DISARM。
- 测试前确认 `MPCA_PC_EN` 档位含义，避免误设为 2 导致未预期触发。

---

## 三、任务 3：YOLO 视觉 YAW 对齐与地面站集成

### 3.1 目标

修复 YOLO 链路问题、标定并补偿视觉延迟、将 YOLO 检测/对齐状态集成到地面站、实现手动测试。

### 3.2 涉及文件

- `YOLO4SEU_MD/rtsp_pillar_node.py`
- `YOLO4SEU_MD/pillar_detector/rtsp_detector.py`
- `YOLO4SEU_MD/utils/rtsp_capture.py`
- `ground_station/scripts/control_ground_station_ros.py`
- `ground_station/src/main.py`（后续集成，本次先不修改）

### 3.3 步骤

#### 步骤 1：修复已知 YOLO bug

已修复：
- ✅ `rtsp_pillar_node.py` 初始化代码位置错误
- ✅ 相机参数统一为 640×360（与 SIYI RTSP 16:9 源保持一致，避免画面拉伸）

待检查/修复：

1. **rtsp_detector.py 中 `confs` 索引风险**
   - 当前 `confs[i]` 与 tracked 目标顺序不一定一致（SORT 可能重排顺序）。
   - 修复：在 `detections_input` 构建时就保存 conf，跟踪后用 bbox 匹配 conf。

2. **FFmpeg 捕获异常处理**
   - `RTSPVideoCapture.read()` 在断流时可能返回 None 或卡死。
   - 增加超时重连机制。

3. **YOLO 推理时间戳**
   - 当前发布 `/yolo/detection_image` 使用 `rospy.Time.now()`，但图像是过去时刻。
   - 应记录图像捕获时间戳并携带到 ROS Image header。

#### 步骤 2：视觉延迟标定

1. 在摄像头前放置一个可精确触发的事件（如 LED 灯开关、秒表、或机械运动）。
2. 同时记录：
   - 摄像头原始图像时间（ROS Image header）
   - `rtsp_pillar_node` 发布 `/yolo/detection_image` 的时间
   - `/yolo/yaw_aligned` 的发布时间
3. 计算端到端延迟：
   ```
   T_delay = T_yolo_publish - T_image_capture
   ```
4. 分别标定：
   - RTSP 编码/传输延迟
   - FFmpeg 解码延迟
   - YOLO 推理延迟
   - SORT 跟踪延迟

标定脚本示例：

```python
# 记录 /yolo/detection_image header.stamp 与当前时间差
import rospy
from sensor_msgs.msg import Image
rospy.init_node('latency_check')
delays = []
def cb(msg):
    delays.append((rospy.Time.now() - msg.header.stamp).to_sec())
rospy.Subscriber('/yolo/detection_image', Image, cb)
rospy.sleep(10)
print(f'mean={sum(delays)/len(delays)*1000:.1f}ms, max={max(delays)*1000:.1f}ms')
```

#### 步骤 3：延迟补偿策略

根据标定结果选择策略：

| 延迟量级 | 策略 |
|----------|------|
| < 50ms | 直接用于 YAW 对齐，无需补偿 |
| 50~150ms | 使用 MAVROS 速度预估当前目标位置，补偿 yaw_rate |
| > 150ms | 优先降低延迟（FFmpeg 参数、YOLO 轻量化、减少缓冲） |

补偿实现思路：

```python
# 在 rtsp_pillar_node.py 中
# 订阅 /mavros/local_position/velocity_local 或 Pose
# 用目标当前角速度估算 T_delay 后的目标方位
# yaw_rate = -Kp * (pixel_error + delay_compensation)
```

#### 步骤 4：YAW 对齐参数实机调优

在悬停状态下：

1. 手动将无人机偏航一定角度，正对柱子。
2. 启动 `rtsp_pillar_node`。
3. 通过地面站或 `rostopic pub` 发布目标锁定或观察自动对齐。
4. 调整参数：
   - `Kp`：过大振荡，过小响应慢
   - `deadzone`：建议 0.05~0.10
   - `max_yaw_rate`：建议 0.3~0.5 rad/s
   - `align_hold_frames`：建议 5~10
5. 记录对齐过程：
   ```bash
   rosbag record /yolo/pixel_error /yolo/yaw_aligned /mavros/setpoint_velocity/cmd_vel /mavros/local_position/pose
   ```
6. 分析 rosbag，评估收敛时间和超调。

#### 步骤 5：YOLO 功能接入地面站

当前 `control_ground_station_ros.py` 已订阅：
- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/statustext/recv`
- `/mavros/battery`

需要新增：

1. 订阅 `/yolo/yaw_aligned`（Bool）
2. 订阅 `/yolo/pixel_error`（Float32）
3. 订阅 `/yolo/detections_info`（String / JSON）
4. 发布 `/yolo/lock_target`（Int32）
5. 在地面站界面增加：
   - 视觉对齐状态指示灯
   - pixel_error 数值显示
   - "启动 YOLO" / "停止 YOLO" 按钮
   - 目标 ID 列表 / 锁定按钮

实现时保持当前地面站 ROS-based 架构不变。

#### 步骤 6：手动测试完整视觉引导流程

1. 启动地面站 + MAVROS + VRPN。
2. 点击"启动 YOLO"按钮，启动 `rtsp_pillar_node`。
3. 手动起飞悬停。
4. 将无人机大致对准柱子方向。
5. 观察地面站：
   - 是否检测到柱子
   - pixel_error 是否收敛到 0
   - yaw_aligned 是否变为 True
6. 确认对齐稳定后，手动前推接近柱子。
7. 安全降落，分析 rosbag。

### 3.4 验收标准

- [ ] YOLO 检测在实机 RTSP 链路上稳定运行，帧率 ≥ 15fps
- [ ] 端到端视觉延迟已标定并记录
- [ ] YAW 对齐在悬停状态下能在 5 秒内收敛，无持续振荡
- [ ] 地面站能显示 `/yolo/yaw_aligned` 和 `/yolo/pixel_error`
- [ ] 地面站能发送 `/yolo/lock_target` 锁定目标
- [ ] 手动测试完成至少 3 次成功对齐

### 3.5 安全要求

- YAW 对齐测试时保持足够高度，避免旋转时碰撞。
- 视觉 YAW 输出的是角速度 setpoint，需确认与 OFFBOARD 其他 setpoint 不冲突。
- 首次测试建议关闭前向速度，仅做原地旋转对齐。

---

## 四、三任务的执行顺序与依赖关系

```
任务 1：参数调优
    │
    ▼
任务 3：YOLO 对齐与地面站集成
    │
    ▼
任务 2：接触检测标定与自主栖停
```

**原因：**

1. 参数调优是基础，所有后续任务都需要稳定飞行的平台。
2. YOLO 对齐是栖停的前提（需要先对准柱子才能稳定接触）。
3. 自主栖停是最终集成，依赖前两者的稳定性。

**实际可并行：**

- 任务 1 的 EKF2 标定可以与任务 3 的延迟标定共享部分数据采集。
- 任务 2 的"阶段 0：确保 ulog 记录数据"可以与任务 1 的飞行测试同步完成。

---

## 五、推荐的首轮试飞安排（示例）

### Day 1：参数调优 — EKF2 + 位置控制

- 上午：标定 `EKF2_EV_DELAY`，记录 VRPN/MAVROS 时间戳。
- 下午：试飞 `MPCA_MODE=0/2`，调 `MPC_XY_*` 和 `MPC_Z_*`。
- 晚上：分析 ulog，更新参数。

### Day 2：参数调优 — 姿态控制 + 变形稳定性

- 上午：调 `MC_*RATE_*` 参数。
- 下午：变形飞行测试，验证 LUT 实时更新稳定性。
- 晚上：形成参数基线文档。

### Day 3：YOLO 链路修复与延迟标定

- 上午：修复 `rtsp_detector.py` conf 索引、检查 FFmpeg 异常处理。
- 下午：标定视觉延迟，调整 `rtsp_pillar_node` 参数。
- 晚上：地面站接入 YOLO 状态显示。

### Day 4：YAW 对齐手动测试

- 上午：悬停状态下测试 YAW 对齐收敛性。
- 下午：手动前推接近柱子，观察对齐保持效果。
- 晚上：分析 rosbag，调参。

### Day 5：接触检测数据采集合

- 上午：确认 ulog 记录完整所需数据。
- 下午/晚上：手动飞行完成 3~5 次接触-栖息流程，下载 ulog。

### Day 6：接触检测标定

- 全天：分析 ulog，标定阈值，更新 `MPCA_PC_*` 参数。
- 编译刷写 v6c。

### Day 7：自主栖停"仅记录"测试

- 全天：手动飞行测试接触判断，验证无漏检/误检。

### Day 8：完整自主栖停验证

- 全天：启用 `MPCA_PC_EN=1`，程序自动执行栖停，人工监控。

---

## 六、需要新建的辅助文件

| 文件 | 用途 |
|------|------|
| `PX4/SEU_MD_PX4/TUNING_FLIGHT_LOG.md` | 记录每次参数变更及试飞结果 |
| `PX4/SEU_MD_PX4/PERCHING_CALIBRATION_LOG.md` | 记录每次手动栖息数据及阈值标定结果 |
| `YOLO4SEU_MD/scripts/measure_yolo_latency.py` | 标定视觉延迟 |
| `YOLO4SEU_MD/scripts/tune_yaw_align.py` | 分析 YAW 对齐 rosbag |

---

## 七、关键命令速查

```bash
# 编译并刷写 v6c
cd ~/Projects/PX4/SEU_MD_PX4
make px4_fmu-v6c_default -j$(nproc)
# QGC 自定义固件刷写 或：
python3 Tools/px_uploader.py --port /dev/ttyACM0 build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4

# 启动地面站
cd ~/Projects/ground_station
./start_ground_station.sh

# 启动 YOLO 节点
roslaunch YOLO4SEU_MD rtsp_pillar.launch rtsp_url:=rtsp://192.168.144.25:8554/main.264

# 记录栖停数据
rosbag record /mavros/local_position/pose /mavros/local_position/velocity_local \
              /mavros/setpoint_position/local /mavros/state \
              /yolo/yaw_aligned /yolo/pixel_error /yolo/detections_info \
              /vrpn_client_node/Tracker1/pose

# 分析 ulog
python3 -m pyulg info /path/to/log.ulg
```

---

## 八、风险管理

| 风险 | 缓解措施 |
|------|---------|
| 调参导致振荡/失控 | 每次只改一个参数，幅度 ≤20%，保留遥控器接管 |
| 自主栖停误触发 | 阶段 4 先"仅记录不动作"验证 |
| 视觉延迟过大 | 优先降低 FFmpeg 缓冲，必要时换用更低分辨率 |
| 接触时损坏机体 | 先用柔性杆/低速测试 |
| 数据记录不全 | 每次飞行前检查 ulog topics 和 rosbag |

---

*本计划为动态文档，每次试飞后应根据实际结果更新参数和下一步重点。*
