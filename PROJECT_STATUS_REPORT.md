# Huaqiccc 变形四旋翼项目 — 综合进展报告 v2

> 生成时间：2026-05-23  
> 基于代码审查、构建验证与仿真实验数据分析  
> 涵盖修复：MPC H 矩阵、hover thrust 传递、integrator、MAVROS 参数同步、perching 失控、Nuttx 兼容性

---

## 目录

1. [执行摘要](#1-执行摘要)
2. [回顾性测试方法（新对话快速上手）](#2-回顾性测试方法新对话快速上手)
3. [PX4 裁剪与系统精简](#3-px4-裁剪与系统精简)
4. [变形仿真实现](#4-变形仿真实现)
5. [三种控制方法实现状态](#5-三种控制方法实现状态)
6. [接触检测实现（历史：GMO/IMU-ICD 尝试）](#6-gmo外力估计器实现)
7. [栖停（Perching）任务实现](#7-栖停perching任务实现)
8. [Bug 修复记录](#8-bug-修复记录)
9. [交叉问题与集成风险](#9-交叉问题与集成风险)
10. [验证方式与实验记录](#10-验证方式与实验记录)
11. [后续工作建议](#11-后续工作建议)

---

## 1. 执行摘要

本项目基于 **PX4 v1.14.3** 开发了一款名为 **huaqiccc** 的变形四旋翼无人机系统。核心特性包括：

- **机械变形**：双臂可在 `[0, -0.50] rad` 范围内展开，改变电机位置与惯性特性
- **实时控制分配**：根据机臂角度 LUT 重建 6×4 执行器效率矩阵
- **三种先进控制**：增益调度 PID（GS-PID）、LQR、线性 MPC
- **外力感知**：基于位置/姿态的接触检测（GMO/IMU-ICD 在实机无效，已弃用）
- **栖停能力**：检测到稳定接触后自动进入 CONTACT → COMPLIANT → GRASP → RAMP_DOWN

**整体完成度评估：**

| 模块 | 完成度 | 状态 |
|------|--------|------|
| PX4 系统裁剪 | ~95% | ✅ 编译通过，FLASH 降至 74.3% |
| 变形仿真（SITL） | ~90% | ✅ 机臂可控，LUT 实时更新矩阵 |
| 三种控制方法 | ~90% | ✅ PID/LQR/MPC 飞行验证通过 |
| 接触检测 | ~90% | ✅ 新位置/姿态检测器实机灵敏度已验证（2026-06-14） |
| 栖停任务 | ~95% | ✅ 端到端仿真验证通过；✅ 实机 `MPCA_PC_EN=2` CONTACT + 自动收臂已稳定（2026-06-23）；⏳ 实机 THROTTLE_RAMP → DISARM 待验证 |

---

## 2. 回顾性测试方法（新对话快速上手）

当新对话需要快速回顾和验证当前项目状态时，按以下顺序执行：

### 2.1 代码状态快速检查（30 秒）

```bash
# 检查关键修复是否仍在位
grep "_mpc_alpha{20.0f}" src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.hpp
grep "setHoverThrust" src/modules/mc_pos_control/MulticopterPositionControl.cpp
grep "CONFIG_MODULES_EXTERNAL_FORCE_ESTIMATOR" boards/px4/sitl/default.px4board
grep "Parameter sync complete" ~/huaqiccc_perching_test.py
```

### 2.2 编译验证（2–3 分钟）

```bash
# SITL
cd /home/a/PX4-Autopilot && make px4_sitl_default -j$(nproc)

# V6C 实机
cd /home/a/PX4-Autopilot && make px4_fmu-v6c_default -j$(nproc)
```

### 2.3 SITL 飞行测试（每种模式 ~90 秒）

```bash
# 简化飞行测试（圆轨迹）
bash ~/run_simplified_test.sh 0   # 原始 PID
bash ~/run_simplified_test.sh 1   # GS-PID
bash ~/run_simplified_test.sh 2   # LQR
bash ~/run_simplified_test.sh 3   # MPC

# 栖停测试
bash ~/run_perching_test.sh 3     # 使用 MPC 模式
```

### 2.4 数据分析（即时）

```bash
# 查看最新日志
ls -lt ~/huaqiccc_logs/*.csv | head -5

# 快速分析误差（示例）
python3 -c "
import csv
rows = list(csv.DictReader(open('~/huaqiccc_logs/huaqiccc_flight_with_algo_...csv')))
last = rows[-60:]
print('err_x:', sum(abs(float(r['err_x'])) for r in last)/60)
"
```

### 2.5 关键文件速查清单

| 模块 | 核心文件 |
|------|----------|
| 变形 + 控制分配 | `ControlAllocator.cpp`, `huaqiccc_motor_lut.hpp`, `mavlink_receiver.cpp` |
| 三种控制器 | `AdvancedPositionControl.cpp/.hpp`, `MulticopterPositionControl.cpp` |
| GMO | `external_force_estimator.cpp/.h` |
| 栖停仿真 | `perching_pole.world`, `mavros_posix_sitl_perching.launch` |
| 测试脚本 | `~/run_simplified_test.sh`, `~/huaqiccc_simplified_flight_test.py`, `~/run_perching_test.sh`, `~/huaqiccc_perching_test.py` |

---

## 3. PX4 裁剪与系统精简

### 3.1 实现概述

为节省 Pixhawk V6C 的 FLASH 空间（从 98.6% 降至 74.3%），项目对 PX4 进行了深度裁剪：

- **删除机型**：固定翼、VTOL、直升机、无人车、UUV、飞艇（~49 个 airframe）
- **删除模块**：`FW_ATT_CONTROL`、`VTOL_ATT_CONTROL`、`ROVER_POS_CONTROL`、`UUV_POS_CONTROL` 等
- **删除驱动**：空速计、光流传感器、遥测（FrSky/HoTT）、UAVCAN、相机/云台
- **精简 Commander**：移除 VTOL/固定翼故障检测、quadchute、airspeed 检查
- **精简 Land Detector**：仅保留多旋翼版本

### 3.2 核心文件

| 文件 | 说明 |
|------|------|
| `boards/px4/fmu-v6c/default.px4board` | V6C 硬件裁剪配置 |
| `boards/px4/sitl/default.px4board` | SITL 裁剪配置 |
| `src/modules/commander/` | 移除 VTOL/FW 逻辑（多处修改） |
| `src/modules/land_detector/` | 移除非 MC 检测器 |
| `src/modules/control_allocator/ActuatorEffectiveness/` | 移除非 MC 效率模型 |

### 3.3 验证状态

- **SITL 编译**：`make px4_sitl_default` ✅ 通过（732/732）
- **V6C 编译**：`make px4_fmu-v6c_default` ✅ 通过（324/324）
- **FLASH 占用**：1,459,904 B / 74.25% ✅
- **构建产物时间戳**：2026-05-23 09:23（最新）

> 验证方式：多次完整编译，无错误无警告。

---

## 4. 变形仿真实现

### 4.1 实现概述

变形系统由三个层面协同工作：

#### 层面 A：Gazebo 物理仿真
- **SDF 模型**：`Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/huaqiccc/huaqiccc.sdf`
- **关节结构**：`left_arm_joint`、`right_arm_joint`（revolute 类型）
- **电机布局**：4 个电机分别挂载在左右臂上，随臂转动改变位置
- **碰撞几何**：使用 STL 网格
- **质量参数**：总质量 1.26064 kg（SDF 中定义）

#### 层面 B：ROS 机臂控制器
- **节点**：`huaqiccc_arm_controller.py`（位于 `~/catkin_ws/src/`）
- **控制模式**：`"effort"`（PD 力矩控制）、`"position"`（直接设定关节角度）
- **运行方式**：需手动启动

#### 层面 C：PX4 控制分配实时更新
- **MAVLink 命令**：自定义命令 `31440`
- **uORB 话题**：`huaqiccc_morph_angle`
- **LUT 查表**：`huaqiccc_motor_lut.hpp`
  - 11 个离散角度点（0.0 ~ -0.50 rad，步长 0.05）
  - 每个点存储 4 个电机的 `(px, py, pz)`
- **矩阵重建**：`ControlAllocator.cpp` 订阅 `huaqiccc_morph_angle`，10 Hz 速率限制下重建 6×4 效率矩阵

### 4.2 核心文件

| 文件 | 说明 |
|------|------|
| `huaqiccc.sdf` / `huaqiccc2.sdf` | Gazebo 模型定义 |
| `huaqiccc_motor_lut.hpp` | 电机位置 LUT |
| `ControlAllocator.cpp` | 矩阵重建逻辑 |
| `mavlink_receiver.cpp` | 31440 命令处理 |
| `huaqiccc_arm_controller.py` | ROS 机臂控制器 |
| `huaqiccc_morphing_demo.py` | 独立变形演示脚本 |

### 4.3 已知问题

1. **ROS 桥缺失**：`huaqiccc/arm_angle` uORB → ROS 未桥接，Python 测试脚本无法直接读取 PX4 内部状态
2. **SITL 与实机 KM 差异**：实机 airframe 使用 `+0.06`，SITL 用 `{-0.05, +0.05, +0.05, -0.05}`
3. **臂控制器未自动启动**：`run_simplified_test.sh` 未启动 `huaqiccc_arm_controller`，变形仅通过 MAVLink 31440 命令影响 PX4 控制分配

### 4.4 验证状态

- **SITL 飞行测试**：✅ 通过，31440 命令成功率 19/19
- **控制分配矩阵更新**：✅ 代码逻辑完整，LUT 查表与矩阵重建已验证
- **机臂物理运动**：⚠️ 未明确验证 Gazebo 关节是否跟随 31440 命令

> 验证方式：代码审查 + SITL 飞行测试 CSV 分析。

---

## 5. 三种控制方法实现状态

### 5.1 整体架构

`AdvancedPositionControl` 是 `PositionControl` 的封装替代，位于 `src/modules/mc_pos_control/AdvancedPositionControl/`。

```
MPCA_MODE = 0 → 原始 PositionControl (完整委托)
MPCA_MODE = 1 → GS-PID (LUT 插值 + 标准 PID)
MPCA_MODE = 2 → LQR (LUT 插值 + PID 结构，但增益为 LQR 推导)
MPCA_MODE = 3 → MPC (梯度投影 QP，N=5，dt=0.05s)
```

### 5.2 Mode 1：增益调度 PID（GS-PID）

- **状态**：✅ 完整实现，飞行验证通过
- **LUT**：11 点，角度 `0.0 → -0.50` rad
- **插值**：线性 `lerp`
- **增益变化**：XY 方向增益从基准增加 ~20-25%（补偿展开后惯性增大）
- **飞行数据**（2026-05-23，修复后）：
  - `err_x` mean=2.05cm, max=3.42cm（最后 3s）
  - `err_y` mean=0.22cm, max=0.50cm
  - `err_z` mean=13.01cm, max=15.32cm

### 5.3 Mode 2：LQR 增益调度

- **状态**：✅ 完整实现，飞行验证通过
- **理论推导**：连续时间双积分器 LQR，解析公式求 K
- **实际执行**：使用 LQR 推导的增益放入**相同 PID 结构**中
- **注意**：这不是纯状态反馈 LQR，而是“LQR 调参的 PID”
- **飞行数据**（历史）：`err_x` mean=0.056m, max=0.187m（三种模式中最佳）

### 5.4 Mode 3：线性 MPC

- **状态**：✅ 实现完整，Bug 已修复，飞行验证通过
- **求解器**：梯度投影 QP
  - `N=5`, `dt=0.05s`, `max_iter=20`, `tol=1e-3`
- **已修复的 Bug**：
  1. **H 矩阵对角错误**：原值 `1.010375`（含 `+I` 偏移），已修正为 `0.020375`
  2. **Alpha 过小**：`_mpc_alpha` 从 `0.978f` 提升到 `20.0f`，匹配修正后的 H 矩阵特征值
  3. **Z 轴约束过紧**：`_mpc_u_max_z` / `_mpc_u_min_z` 从 `±2.0` 扩展到 `±8.0`
  4. **积分器增益不足**：`_mpcControl()` 中积分器更新从 `* dt * 0.1f` 恢复为完整 `* dt`
  5. **Hover thrust 未传递**：在 `MulticopterPositionControl.cpp` 中添加 `_advanced_control.setHoverThrust()` 调用
- **当前矩阵**：
  ```cpp
  H = [[0.020375, 0.0079, 0.005525, 0.00335, 0.001475],
       [0.0079,   0.0171, 0.005075, 0.00315, 0.001425],
       ...]
  _mpc_alpha = 20.0f
  _mpc_u_max_z = 8.0f
  ```
- **飞行数据**（修复后，MPCA_MODE=3）：
  - XY：`err_x` mean=1.5cm, `err_y` mean=0.9cm（最后 3s）
  - Z：`err_z` mean=5.3cm（仍有 ~5cm 稳态误差，由积分器持续补偿中）
  - 着陆：✅ 正常降落，无失控

### 5.5 核心文件

| 文件 | 说明 |
|------|------|
| `AdvancedPositionControl.cpp/.hpp` | 三种模式的完整实现 |
| `MulticopterPositionControl.cpp` | MPCA_MODE 分支、栖停逻辑、hover thrust 传递 |
| `mc_pos_control_params.c` | `MPCA_MODE` 参数定义 |

### 5.6 验证状态

| 模式 | 编译 | SITL 飞行 | 数据质量 |
|------|------|-----------|----------|
| 0 (PID) | ✅ | ✅ | ✅ 基准优良 |
| 1 (GS-PID) | ✅ | ✅ | ✅ 优良 |
| 2 (LQR) | ✅ | ✅ | ✅ 最佳 |
| 3 (MPC) | ✅ | ✅ | ✅ 优良（修复后） |

> 验证方式：每种模式独立运行 `run_simplified_test.sh`，收集 CSV，用 Python/pandas 分析误差统计。

---

## 6. 接触检测实现（已重构，2026-06-14）

### 6.1 历史说明：IMU-ICD / GMO 尝试

`external_force_estimator` 模块曾尝试用双窗口广义动量观测器（GMO）/ IMU 脉冲接触检测（IMU-ICD）
实现无外部传感器的接触检测。该思路受北航墙面栖息工作启发，代码已完成并在 SITL 中运行。

**然而，2026-06-13 实机手动栖停日志显示**：
- 无人机悬停时原始 IMU 加速度模值已约 **25 m/s²**；
- 与杆接触前后，IMU 加速度模值没有可区分的跳变；
- 因此 IMU-ICD 无法为实机栖停提供可靠的接触信号。

结论：`external_force_estimator` 作为**实机接触检测器是失败的**。代码保留在树中供参考，但：
- `boards/px4/fmu-v6c/default.px4board` 中 `CONFIG_MODULES_EXTERNAL_FORCE_ESTIMATOR=n`
- `rc.mc_apps` 中不再 `external_force_estimator start`
- 相关打印频率已大幅降低

### 6.2 当前方案：位置/姿态接触检测

接触检测现在完全在 `mc_pos_control` 中实现，使用三个容易获取且可靠的信号：

| 信号 | 阈值参数 | 默认值 | 说明 |
|------|----------|--------|------|
| 沿 setpoint 方向的位置误差 | `MPCA_PC_SERR` | 0.05 m | setpoint 在前，实际位置被杆卡住 |
| 沿 setpoint 方向的速度 | `MPCA_PC_SVEL` | 0.10 m/s | 前进速度接近 0 |
| 俯仰角前倾 | `MPCA_PC_PIT_THR` | -5.0° | 机头下压顶杆 |
| 连续满足时间 | `MPCA_PC_DUR_THR` | 0.30 s | 过滤姿态瞬时波动 |

检测方向取 **setpoint − current_position 的水平向量**，自动对齐实际前进方向，不再硬编码 NED Y。

### 6.3 核心文件

| 文件 | 说明 |
|------|------|
| `src/modules/mc_pos_control/MulticopterPositionControl.cpp/.hpp` | 位置/姿态接触检测 + Perching FSM |
| `src/modules/mc_pos_control/mc_pos_control_params.c` | `MPCA_PC_*` 参数 |
| `src/modules/external_force_estimator/` | [保留，已弃用于实机] IMU-ICD 尝试 |
| `msg/ContactState.msg` | uORB 消息定义（保留兼容） |

### 6.4 参数模式

| `MPCA_PC_EN` | 行为 |
|--------------|------|
| 0 | OFF：不检测、不记录 |
| 1 | DETECT：检测并打印 `CONTACT_DETECTED` 日志，**不进入 Perching FSM** |
| 2 | FULL：检测到后进入 CONTACT → COMPLIANT → RAMP_DOWN → PERCHED |

实机 airframe 默认 `MPCA_PC_EN=1`，便于先验证检测时机再开启全自动栖停。

### 6.5 验证状态

- **离线日志分析**：✅ 两次手动栖停日志（2026-06-13）中，组合判据零误检、检出率 90~100%
- **固件编译**：✅ v6c 编译通过，`px4_fmu-v6c_default.px4` 已生成
- **实机验证**：✅ 2026-06-14 手动栖停测试，检测灵敏度合适，遥测链路正常

> 验证方式：pandas 日志分析 + 固件编译 + 实机 MAVLink 日志对齐。

---

## 7. 栖停（Perching）任务实现

### 7.1 实现概述

栖停逻辑集成在 `MulticopterPositionControl` 中：

1. **触发条件**：`MPCA_PC_EN=2` 且位置/姿态接触检测器输出 `CONTACT_DETECTED`
2. **执行动作**：
   - 记录接触点 `_perching_contact_x`
   - 进入 Perching FSM：CONTACT → COMPLIANT → RAMP_DOWN → PERCHED
   - COMPLIANT 阶段软化位置 P 增益、重置积分器、使用弹簧模型维持竖直推力
   - RAMP_DOWN 阶段指数衰减推力到 0
3. **安全退出**：高度下降 > 0.3 m 或 COMPLIANT 超时 20 s 未抓稳 → 回到 NONE

### 7.2 仿真环境

- **世界**：`perching_pole.world`（杆位于 x=5.0m）
- **杆模型**：`perching_pole.sdf`
  - 圆柱，半径 0.04m，长度 5.0m
  - 摩擦系数 μ=0.9，kp=1e5，kd=10
  - 几乎无弹性（restitution=0.01）
- **碰撞检测**：`mc_pos_control` 内位置/姿态接触检测（位置误差 + 低速 + pitch）

### 7.3 测试脚本

- **`huaqiccc_perching_test.py`**：
  - 起飞到 2.5m → 展开机臂 → 飞至 x=4.0m（接近杆）→ 慢速前推（0.25 m/s）→ 检测 stall → 收拢机臂 → 降落
  - v3 修复：MAVROS 参数同步 + 变形期间位置保持 + OFFBOARD 守卫

### 7.4 端到端验证结果（2026-05-23）

```
[OK] Parameter sync complete, 844 params received
[OK] Set MPCA_MODE
[HOVER] 8.0s at (0.0, 0.0, 2.5)
[MORPH] Slow morph to -0.30 rad over 3.0s (hold at 0.0,0.0,2.5)
[APPROACH] (0.0,0.0) -> (4.0,0.0) over 10.0s
[PUSH] From x=4.00 toward x=5.00 at 0.25m/s
  [STALL] act_x=4.93, sp_x=4.98, s=0.91
[CONTACT] Detected at t=3.65s | stalled=True
[GRASP] Maintaining push at x=5.25 while closing arms...
[LAND] Landing
[SAVE] perching_test_20260523_103802.csv
```

- **接触检测**：✅ 通过 stall detection 成功检测（act_x=4.93m，接近 pole_x=5.0m）
- **变形期间稳定性**：✅ 无失控、无下降
- **轨迹跟踪**：✅ Hover err_x=8.39cm, Approach err_x=12.96cm（可接受）
- **降落**：✅ 正常

### 7.5 验证状态

- **逻辑代码**：✅ 完整
- **仿真环境**：✅ 世界、杆模型、启动文件均存在
- **端到端测试**：✅ 成功执行（2026-05-23），完整飞行 + 接触 + 收拢 + 降落
- **实机接触检测**：✅ 2026-06-14 手动栖停测试，`MPCA_PC_EN=1` 遥测链路正常，检测灵敏度合适

> 验证方式：代码审查 + SITL 端到端飞行测试 + 实机日志离线分析 + MAVLink 日志对齐。

---

## 8. Bug 修复记录

### Bug #1：MPC H 矩阵对角含 `+I` 偏移

| 项目 | 内容 |
|------|------|
| **现象** | MPC 模式收敛极慢，Z 轴需 36–40s 才能从 1.11m 爬到 2.0m；着陆阶段失控坠毁 |
| **根因** | `mpc_H_init` 对角值被错误地写为 `I + R_bar`（如 `1.010375 = 1.0 + 0.010375`），正确的 QP Hessian 应仅为 `R_bar` 贡献 |
| **修复** | 将对角值修正为 `0.020375, 0.0171, 0.014625, 0.01275, 0.011275` |
| **文件** | `AdvancedPositionControl.cpp` |
| **验证** | SITL 飞行测试：Z 轴立即起飞，hover 误差从 >50cm 降至 ~5.3cm |

### Bug #2：MPC `_mpc_alpha` 过小

| 项目 | 内容 |
|------|------|
| **现象** | H 矩阵修复后，求解器步长相对于特征值过小，收敛仍慢 |
| **根因** | 原 `alpha=0.978` 是基于 `λ_max≈1.0`（错误 H 矩阵）设计的；修正后 `λ_max≈0.02`，需要更大的 alpha |
| **修复** | `_mpc_alpha` 从 `0.978f` 提升到 `20.0f`（稳定上界约 100） |
| **文件** | `AdvancedPositionControl.hpp` |
| **验证** | SITL 飞行测试：求解器响应及时，XY 误差 <2cm |

### Bug #3：MPC Z 轴约束过紧

| 项目 | 内容 |
|------|------|
| **现象** | 即使 H 矩阵和 alpha 修正后，Z 轴仍无法产生足够推力起飞 |
| **根因** | `_mpc_u_max_z = 2.0f` 太小，不足以克服重力加速度（需 ~9.8 m/s²） |
| **修复** | `_mpc_u_max_z` / `_mpc_u_min_z` 从 `±2.0` 扩展到 `±8.0` |
| **文件** | `AdvancedPositionControl.hpp` |
| **验证** | SITL 飞行测试：起飞立即成功，无延迟 |

### Bug #4：MPC 积分器增益被削弱 10 倍

| 项目 | 内容 |
|------|------|
| **现象** | Z 轴存在 ~8.5cm 稳态误差，无法完全消除 |
| **根因** | `_mpcControl()` 中积分器更新写为 `* dt * 0.1f`，导致积分速度仅为正常的 1/10 |
| **修复** | 恢复为完整 `* dt`；`_velocityControl()` 中仍保留 `0.1f`（模式 1/2 不需要大积分） |
| **文件** | `AdvancedPositionControl.cpp`（第 492 行） |
| **验证** | SITL 飞行测试：Z 稳态误差从 ~8.5cm 降至 ~5.3cm |

### Bug #5：Hover Thrust 未传递给 Advanced 控制器

| 项目 | 内容 |
|------|------|
| **现象** | 所有 advanced 模式（1/2/3）hover thrust 使用默认值 0.5，与 SDF 实际值 0.4375 不符，导致 Z 轴持续偏差 |
| **根因** | `MulticopterPositionControl.cpp` 中只调用了 `_control.setHoverThrust()`，未调用 `_advanced_control.setHoverThrust()` |
| **修复** | 在初始化处和非飞行更新处，同时调用 `_advanced_control.setHoverThrust(_param_mpc_thr_hover.get())` |
| **文件** | `MulticopterPositionControl.cpp`（第 262、496 行） |
| **验证** | SITL 飞行测试：Z 轴偏差减小，变形后漂移降低 |

### Bug #6：MAVROS 参数同步缺失（Simplified Test）

| 项目 | 内容 |
|------|------|
| **现象** | `run_simplified_test.sh` 运行时出现 `[ERROR] PR: Unknown parameter to set: MPCA_MODE` / `MPC_XY_P` |
| **根因** | Python 脚本在 FCU 连接后立即调用 `param_set()`，此时 MAVROS 尚未从 PX4 下载完参数缓存 |
| **修复** | 添加 `ParamPull` 服务调用，等待 844 个参数同步完成后再设置参数 |
| **文件** | `huaqiccc_simplified_flight_test.py` |
| **验证** | SITL 飞行测试：`Parameter sync complete, 844 params received`，无报错 |

### Bug #7：MAVROS 参数同步缺失 + 变形失控（Perching Test）

| 项目 | 内容 |
|------|------|
| **现象** | `run_perching_test.sh` 运行时同样出现 `Unknown parameter` 错误；变形时无人机快速下降，随后以极快速度飞行，轨迹完全偏离 |
| **根因 1** | 同 Bug #6：MAVROS 参数缓存未同步 |
| **根因 2** | `_morph_arm_slowly()` 在变形期间只发送 arm angle topic，不发送 position setpoint，导致 PX4 OFFBOARD 模式超时失效 |
| **修复 1** | 添加 `ParamPull` 同步等待 |
| **修复 2** | `_morph_arm_slowly()` 始终发送位置保持 setpoint；变形前后添加 `_ensure_offboard()` 检查和 40 个 setpoint burst |
| **文件** | `huaqiccc_perching_test.py` |
| **验证** | SITL 端到端测试：参数设置成功，变形期间位置稳定，approach/push/retreat/land 全部正常完成 |

### Bug #8：Nuttx 不兼容（`std::nth_element` + 大栈数组）

| 项目 | 内容 |
|------|------|
| **现象** | `make px4_fmu-v6c_default` 编译失败，`external_force_estimator` 模块报错 |
| **根因** | Nuttx 无完整 C++ STL（无 `std::nth_element`），且栈限制 2048B，4000B 临时数组导致栈溢出 |
| **修复** | 自定义 `simple_nth_element`（选择排序至 k）；将中值临时数组声明为 `static` |
| **文件** | `external_force_estimator.cpp` |
| **验证** | V6C 编译通过（324/324），固件 `.px4` 产物包含 `external_force_estimator_main` 符号 |

---

## 9. 交叉问题与集成风险

### 9.1 剩余已知问题

| 优先级 | 问题 | 影响 | 修复复杂度 |
|--------|------|------|-----------|
| 🟡 P1 | `px4_msgs` 未安装 → Python 无法读取 GMO 数据 | 栖停脚本看不到 efo_mag/cstate | 低（`pip install px4-msgs` 或改用 MAVLink） |
| 🟡 P1 | ROS uORB → `/huaqiccc/arm_angle` 桥缺失 | 脚本读取不到 PX4 内部 arm angle | 中（需 uORB→ROS 桥或 MAVLink 流） |
| 🟢 P2 | 机臂控制器未随测试脚本自动启动 | SITL 中 Gazebo 关节可能不跟随 31440 | 低（launch 文件加节点） |
| 🟢 P2 | Mode 2 LQR 实为“LQR 增益 PID” | 非纯 LQR，理论收益未完全发挥 | 中（需重写为纯状态反馈） |
| 🟢 P2 | MPC Z 方向 ~5cm 稳态误差 | 积分器或前馈可进一步优化 | 低（调参或加 feedforward） |
| 🟢 P2 | 变形后 ~5cm XY 漂移 | 物理 CoP 偏移导致，需前馈补偿 | 中（需建模 CoP 位移） |

### 9.2 架构依赖图

```
[变形命令 31440] → [mavlink_receiver] → [huaqiccc_morph_angle uORB]
                                                  ↓
[ControlAllocator] ←  LUT 查表 ← [huaqiccc_motor_lut.hpp]
                                                  ↓
[AdvancedPositionControl] ← MPCA_MODE (0/1/2/3)
         ↓
[MC Attitude/Rates] → [Gazebo SITL]
         ↑
[External Force Estimator] ← 车辆状态 (IMU, EKF)  ✅ 已编译进固件
         ↓
[contact_state uORB] ──────→ ❌ px4_msgs 未桥接（Python 端不可见）
         ↓
[MulticopterPositionControl] ← 栖停逻辑
         ↓
[前推 0.25m 设定点覆盖] ──────→ ✅ 端到端验证通过
```

---

## 10. 验证方式与实验记录

### 10.1 验证方法总结

| 验证层级 | 方法 | 覆盖范围 |
|----------|------|----------|
| 代码审查 | 直接阅读源码 + grep 搜索 | 全部模块 |
| 编译验证 | `make px4_sitl_default` / `make px4_fmu-v6c_default` | SITL + V6C |
| 单元测试 | PX4 原有 `PositionControlTest` | 仅原始 PID |
| SITL 飞行测试 | `run_simplified_test.sh` + CSV 分析 | 模式 0/1/2/3 |
| SITL 栖停测试 | `run_perching_test.sh` + CSV 分析 | 端到端 perching |
| 日志分析 | Python/pandas 统计误差 | 模式 0/1/2/3 + perching |
| 构建产物检查 | `.ninja_log`, `boardconfig`, `.a` 文件, `nm` 符号 | 外部模块 |

### 10.2 实验记录清单

| 日期 | 实验内容 | 结果 |
|------|----------|------|
| 2026-05-19 | 模式 0 基线飞行 | ✅ `err_x` mean=0.065m |
| 2026-05-19 | 模式 1 GS-PID | ✅ `err_x` mean=0.060m |
| 2026-05-19 | 模式 2 LQR | ✅ `err_x` mean=0.056m |
| 2026-05-21~23 | 栖停测试（多次） | ❌ `efo_mag=0`, `cstate=-1`, 失控坠毁 |
| 2026-05-23 | 模式 3 MPC（矩阵修正后） | ✅ XY err<2cm, Z err~5.3cm, 着陆正常 |
| 2026-05-23 | 栖停测试 v3（Bug 修复后） | ✅ 端到端通过，stall detection 成功 |
| 2026-05-23 | V6C 编译验证（Nuttx 修复后） | ✅ 324/324 通过，固件含 GMO 模块 |

### 10.3 验证局限性

- **无自动回归测试**：每次修改需手动运行 `run_simplified_test.sh`（~90 秒）
- **GMO/IMU-ICD 已弃用**：实机验证无效，当前使用位置/姿态接触检测
- **无 MPC 数值验证**：未用 Python/MATLAB 对比求解器输出
- **uORB 内部状态不可见**：无法直接观察中间变量

---

## 11. 后续工作建议

### 11.1 立即修复（P1 — 数据链路）

1. **启用 `MPCA_PC_EN=2` 进行自主栖停测试**
   - 在人工监控下验证 CONTACT → COMPLIANT → GRASP → RAMP_DOWN 全流程
   - 通过 `/mavros/debug_value/debug_float_array` 实时观察状态

2. **迭代 `MPCA_PC_*` 阈值（如需）**
   - 若出现误触发或漏检，调整 `MPCA_PC_SERR/SVEL/PIT_THR/DUR_THR`

### 11.2 短期完善（P2 — 性能优化）

3. **MPC Z 稳态误差优化**
   - 当前 ~5.3cm，可通过增大积分器上限或添加重力前馈进一步改善

4. **变形漂移前馈补偿**
   - 变形时产生的 ~5cm XY 漂移，可通过 LUT 中预存 CoP 偏移量进行前馈补偿

5. **机臂控制器自动启动**
   - 修改 `mavros_posix_sitl.launch`，添加 `<node pkg="huaqiccc" type="huaqiccc_arm_controller.py" .../>`

### 11.3 中期优化（P2 — 算法深化）

6. **纯 LQR 状态反馈**
   - 将 Mode 2 从"LQR 增益 PID"改为真正的 `u = -K*x` 状态反馈

7. **位置/姿态检测器阈值固化**
   - 根据 2026-06-14 及后续自主栖停测试结果，将最终阈值写入 airframe

### 11.4 实机准备

8. ✅ **V6C 固件刷写验证**（已完成）
9. ✅ **室内动捕定位集成**（已完成）
10. **YOLO 视觉 YAW 对齐与实机 OFFBOARD 协调**
11. **一键栖停任务自动化**

---

## 附录 A：核心文件路径速查

```
# 变形核心
src/modules/control_allocator/ControlAllocator.cpp
src/modules/control_allocator/huaqiccc_motor_lut.hpp
src/modules/mavlink/mavlink_receiver.cpp
ROMFS/px4fmu_common/init.d/airframes/4400_huaqiccc
ROMFS/px4fmu_common/init.d-posix/airframes/4400_gazebo-classic_huaqiccc

# 控制
src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.cpp
src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.hpp
src/modules/mc_pos_control/MulticopterPositionControl.cpp
src/modules/mc_pos_control/MulticopterPositionControl.hpp
src/modules/mc_pos_control/mc_pos_control_params.c

# GMO
src/modules/external_force_estimator/external_force_estimator.cpp
src/modules/external_force_estimator/external_force_estimator.h
msg/ContactState.msg
msg/ExternalForceEstimate.msg

# 栖停仿真
Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/perching_pole.world
Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/perching_pole/pole.sdf
launch/mavros_posix_sitl_perching.launch

# 测试
~/run_simplified_test.sh
~/huaqiccc_simplified_flight_test.py
~/run_perching_test.sh
~/huaqiccc_perching_test.py
~/catkin_ws/src/huaqiccc_arm_controller.py
~/catkin_ws/src/huaqiccc_morphing_demo.py
```

## 附录 B：备份与恢复

所有关键文件的备份位于 `~/huaqiccc_backup_20260523/`，包含：
- `PROJECT_STATUS_REPORT.md`（本报告）
- `BUG_FIX_RECORD.md`（Bug 修复详细记录）
- 修改过的核心源码文件副本
- 测试脚本副本

如需恢复到当前状态，从备份文件夹复制文件即可。

---

*报告结束。*
