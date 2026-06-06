# huaqiccc 变形飞行机器人项目进展总结

> 生成日期：2026-05-27
> 对应PX4版本：v1.14.3
> 项目状态：仿真验证阶段完成，**实机固件已编译并烧写到 Pixhawk 4 (FMU-v5)**

---

## 1. 项目总览

本项目面向电力杆塔等柱状基础设施检测需求，研制了一种通过机身变形实现在柱状结构上栖息作业的飞行机器人。核心创新点包括：

1. **变形机构设计**：一维旋转开合臂，通过丝杆推杆实现左右机身对称展开/收拢
2. **多模式自适应控制**：在单一PX4飞控框架内实现了原始PID → 增益调度PID → LQR → MPC → MPC+微分平坦前馈的完整控制方法链
3. **机载IMU接触检测**：无需额外传感器，仅通过IMU加速度/角速度实现5状态接触检测FSM
4. **自主栖息全流程**：有限状态机统一调度飞行→接近→接触→夹持→栖息→重新起飞

---

## 2. 代码修改清单

### 2.1 新增模块

| 模块路径 | 功能 | 文件数 |
|---------|------|--------|
| `src/modules/mc_pos_control/FlatnessFeedforward/` | 微分平坦映射：从flat output计算推力、姿态、角速度、角加速度 | 3 |
| `src/modules/mc_pos_control/AdvancedPositionControl/` | 增强位置控制器：4种模式（PID/GS-PID/LQR/MPC+FF） | 3 |
| `src/modules/external_force_estimator/` | IMU-based接触检测FSM（IMU-ICD） | 4 |
| `src/modules/control_allocator/huaqiccc_motor_lut.hpp` | 预计算电机位置查找表（11点，步长0.05 rad） | 1 |

### 2.2 修改的PX4核心文件

| 文件 | 修改内容 | 影响范围 |
|------|---------|---------|
| `src/modules/mc_pos_control/CMakeLists.txt` | 添加AdvancedPositionControl和FlatnessFeedforward子目录 | 构建系统 |
| `src/modules/mc_pos_control/MulticopterPositionControl.cpp/hpp` | 集成_advanced_control，传递flatness输入和morph角度 | 位置控制 |
| `src/modules/mc_pos_control/mc_pos_control_params.c` | 新增12个参数：MPCA_MODE, MPCA_MPC_ALPHA, MPCA_MPC_R_DELTA, MPCA_FF_EN, MPCA_FF_BLEND, MPCA_FF_MASS, **MPCA_PC_EN, MPCA_PC_TRIG, MPCA_PC_GATE, MPCA_PC_STALL_D, MPCA_PC_STALL_T, MPCA_PC_SERR, MPCA_PC_SVEL** | 参数系统 |
| `src/modules/mavlink/mavlink_receiver.cpp/h` | **关键修复**：SET_POSITION_TARGET_LOCAL_NED的加速度字段未写入setpoint.acceleration | MAVROS接口 |
| `src/modules/control_allocator/ControlAllocator.cpp` | 拦截morph uORB，从LUT重建控制效率矩阵 | 控制分配 |
| `src/modules/control_allocator/ActuatorEffectivenessRotors.cpp/hpp` | 支持动态旋翼配置 | 控制分配 |
| `msg/HuaqicccMorphAngle.msg` | 变形臂角uORB话题定义 | 通信 |
| `msg/ContactState.msg` | 接触状态话题定义（5状态FSM输出） | 通信 |
| `msg/ExternalForceEstimate.msg` | 外部力估计话题定义 | 通信 |
| `ROMFS/px4fmu_common/init.d-posix/px4-rc.params` | 新增MPCA参数默认值（MPCA_MODE=2 即LQR为默认），**新增EFO_ENABLE, MPCA_PC_EN等栖停参数默认值** | ROMFS |
| `launch/mavros_posix_sitl.launch` | 修复fcu_url为udp://:14540@localhost:14580 | 启动配置 |

### 2.3 新增机架配置文件

| 文件 | 说明 |
|------|------|
| `ROMFS/px4fmu_common/init.d-posix/airframes/4400_gazebo-classic_huaqiccc` | Gazebo SITL机架配置 |
| `ROMFS/px4fmu_common/init.d/airframes/4400_huaqiccc` | SITL机架配置（旧） |
| `ROMFS/px4fmu_common/init.d/airframes/4401_huaqiccc_real` | **实机机架配置（Pixhawk 4 FMU-v5，已烧写验证）** |

### 2.4 测试脚本

| 脚本 | 功能 | 状态 |
|------|------|------|
| `~/run_huaqiccc_test.sh` | 测试套件统一入口，支持 flatness/aggressive/batch/perching 等 | ✅ 可用 |
| `~/huaqiccc_test_suite/flight/flatness_circle.py` | 圆轨迹+变形+速度/加速度前馈测试（v4.1） | ✅ 可用 |
| `~/huaqiccc_test_suite/flight/aggressive_trajectory.py` | 激进轨迹测试，支持--radius/--period | ✅ 可用 |
| `~/huaqiccc_test_suite/perching/pole_collision.py` | **栖停撞击测试：起飞→变形→接近→推杆→Stall检测→夹持** | ✅ 可用 |
| `~/huaqiccc_test_suite/perching/manual_control_gui.py` | 手动控制GUI（含freeze/固定到柱子功能） | ✅ 可用 |
| `~/huaqiccc_test_suite/runners/01_flatness_circle.sh` | MPC+FF 单测试运行脚本 | ✅ 可用 |
| `~/huaqiccc_test_suite/runners/03_aggressive_trajectory.sh` | 激进轨迹运行脚本 | ✅ 可用 |
| `~/huaqiccc_test_suite/runners/22_batch_aggressive_repeated.sh` | 批量对比测试（5算法×2重复） | ✅ 可用 |
| `~/huaqiccc_test_suite/runners/23_mpc_parameter_sweep.sh` | MPC参数调优脚本 | ✅ 可用 |

### 2.5 论文草稿

| 文件 | 说明 |
|------|------|
| `~/PX4-Autopilot/大纲0418/大纲0418.md` | 论文完整大纲（557行） |
| `~/PX4-Autopilot/大纲0418/章节草稿/第2-3章_实机版_逻辑通顺版.md` | 第2-3章初稿（304行，无四级标题） |
| `~/PX4-Autopilot/大纲0418/章节草稿/第2-3章_实机版_初稿.md` | 第2-3章初稿（478行，有四级标题） |
| `~/REPRODUCE.md` | 实验复现指南 |
| `~/PAPER_ANALYSIS.md` | 论文写作分析（文献调研+差距分析） |

---

## 3. 关键Bug修复记录

### Bug 1：MAVROS加速度前馈未传递（致命）

**现象**：Python测试脚本发送的加速度前馈（afx/afy/afz）未到达PX4控制器，导致微分平坦前馈完全失效。

**根因**：`mavlink_receiver.cpp`中，`SET_POSITION_TARGET_LOCAL_NED`消息处理代码仅将afx/afy/afz读取到局部变量，但未复制到`setpoint.acceleration`数组。

**修复**：在`mavlink_receiver.cpp`约997行添加：
```cpp
setpoint.acceleration[0] = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? (float)NAN : target_local_ned.afx;
setpoint.acceleration[1] = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? (float)NAN : target_local_ned.afy;
setpoint.acceleration[2] = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? (float)NAN : target_local_ned.afz;
```

**影响**：修复后，微分平坦前馈可正常接收轨迹加速度信息，MPC+FF模式生效。

### Bug 2：单位混淆导致推力失控（致命）

**现象**：启用flatness feedforward后，无人机瞬间失控、姿态翻转。

**根因**：原始代码直接将flatness输出的`collective_thrust`（单位：N，牛顿）叠加到`_thr_sp`（单位：归一化推力，0-1），导致数值量纲错误。例如：15N的推力被当作15倍的最大推力加入。

**修复**：改为将flatness加速度以m/s²形式注入`_acc_sp`：
```cpp
// 错误做法（已删除）：
// _thr_sp += ff.body_z * ff.collective_thrust * blend;

// 正确做法：
_acc_sp_ff = Vector3f(_flat_output.acc) * _flatness_blend;
ControlMath::addIfNotNanVector3f(_acc_sp, _acc_sp_ff);
// 然后由_accelerationControl()将_acc_sp转换为_thr_sp
```

**影响**：修复后，MPC+FF模式稳定运行，前馈改善15.9%。

### Bug 3：参数未持久化导致测试不一致

**现象**：每次测试后，MPCA_MODE等参数被保存到parameters.bson，下一次测试即使修改了ROMFS默认值，PX4仍读取上一次的保存值。

**根因**：PX4优先读取`build/px4_sitl_default/rootfs/parameters.bson`而非ROMFS默认值。

**修复**：`run_flatness_test.sh`在每次启动前删除：
```bash
rm -f build/px4_sitl_default/rootfs/parameters.bson
rm -f build/px4_sitl_default/rootfs/parameters_backup.bson
```

**影响**：每次测试都从ROMFS默认值开始，保证参数一致性。

### Bug 4：MAVROS端口不匹配

**现象**：MAVROS无法连接PX4 SITL。

**根因**：`mavros_posix_sitl.launch`中`fcu_url`设置为`udp://:14540@localhost:14557`，但PX4 SITL实际监听端口为14580。

**修复**：修改为`udp://:14540@localhost:14580`。

---

## 4. 核心设计决策

### 4.1 控制器架构

```
[Trajectory Setpoint] → [MulticopterPositionControl]
                              │
                    ┌─────────┴─────────┐
                    │                   │
            [Original PID]    [AdvancedPositionControl]
                                    │
                        ┌───────┼───────┐
                        │       │       │
                     Mode=1  Mode=2  Mode=3
                   GS-PID    LQR     MPC
                                    │
                              [FlatnessFeedforward]
```

### 4.2 控制效率矩阵更新流程

```
[MAVROS /h Miracle] → [mavlink_receiver] → [huaqiccc_morph_angle uORB]
                                                          │
                                                          ↓
                                [ControlAllocator] ← reads arm_angle
                                     │
                                     ↓
                         [huaqiccc_get_motor_params LUT]
                                     │
                                     ↓
                         [rebuild effectiveness matrix B(β)]
                                     │
                                     ↓
                         [update _control_allocation instances]
```

### 4.3 IMU-ICD接触检测FSM

```
NO_CONTACT ──impact>a_thr──> IMPACT ──time>t_thr──> CONFIRMED
     ↑                              │                    │
     └──────impact<a_thr×0.2───────┘    gyro<g_thr ───> STABLE
                                               │           │
                                          gyro>g_thr×2.5   │should_close
                                               │           ↓
                                          SLIPPING <──── [触发前推+收拢]
```

### 4.4 Perching触发逻辑

```
[Stall Detected (Y-axis)] / [ContactState == STABLE] + [flying] + [y>gate] + [armed>8s]
                              │
                              ↓
                    [_perching_active = true]
                              │
                              ↓
              [_setpoint.position[1] = _perching_contact_y + 0.05m (CONTACT)]
                              │
                              ↓
                    [机臂收拢指令下发] → [电机逐步降速至关闭]
```

> ✅ **坐标系确认（2026-06-03）**：MAVROS `setpoint_raw/local` **无条件**调用 `ftf::transform_frame_enu_ned()`。Python ENU `(x, y)` → PX4 NED `(position[1], position[0])`。因此 **NED Y 轴 (`position[1]`) 是飞行方向**，所有 perching 逻辑（Stall Detection、setpoint override）正确作用于 `position[1]`。此前 2026-06-01 将 Stall Detection 改到 X 轴的修复是错误的，已 revert。

---

## 5. 实验验证结果

### 5.1 轨迹跟踪对比（SITL，R=1.5m，T=15s）

| 控制器 | 动态XY RMSE | 改善比例 | 一致性（std） |
|--------|-------------|---------|--------------|
| 原始PID | 0.2701 m | — | ±0.0014 |
| 增益调度PID | 0.1007 m | +62.7% | ±0.0022 |
| **LQR** | **0.0740 m** | **+72.6%** | **±0.0005** |
| MPC | 0.1261 m | +53.3% | ±0.0085 |
| MPC+微分平坦前馈 | 0.1060 m | +60.7% | ±0.0009 |

### 5.2 栖停接触检测验证（SITL，Stall Detection）

**测试配置**：
- 杆位置：x=5.0, y=0.0，直径 18cm
- Stall Detection 参数：`MPCA_PC_SERR=0.05`, `MPCA_PC_SVEL=0.10`, `MPCA_PC_STALL_T=1.0`, `MPCA_PC_STALL_D=0.03`
- 触发模式：`MPCA_PC_TRIG=1`（STALL_ONLY，IMU禁用）

| 场景 | STALL_DETECTED 次数 | 结果 |
|------|---------------------|------|
| **有柱子**（`perching_pole.world`） | 1次（触发perching FSM） | ✅ 成功进入 CONTACT → COMPLIANT |
| **无柱子**（`empty.world`） | 0次 | ✅ 假阳性率 **0%** |

**关键发现**：
1. **Stall Detection 在SITL中可靠**：纯位置/速度判断，不依赖Gazebo碰撞响应质量
2. **IMU-ICD在SITL中不可靠**：Gazebo IMU数据缺乏真实碰撞瞬态，STABLE状态难以触发
3. **双触发架构灵活**：`MPCA_PC_TRIG` 参数可在SITL/实机间切换最优检测策略

### 5.3 离线算法评估（102条CSV日志）

对5种候选接触检测算法在既有飞行日志上的评估：

| 算法 | 原理 | 真阳性 | 假阳性 | 结论 |
|------|------|--------|--------|------|
| A | 位置误差阈值 | 低 | **>60%** | 悬停/机动易误触发 |
| B | 速度阈值 | 低 | **>60%** | 同上 |
| C | 位置+速度联合 | 中 | **>60%** | 数据限制，hover漂移误判 |
| D | 电机饱和检测 | 高 | **0%** | 需电机数据，最佳但数据源受限 |
| E | 设定点追踪偏差 | 中 | **>75%** | 接近阶段瞬态误触发 |

**结论**：纯位置/速度方法在既有日志上假阳性率极高（因日志中缺乏真实碰撞数据，主要为hover/接近/机动）。必须在SITL中通过物理碰撞验证。

### 5.4 关键发现（累计）

1. **LQR优于MPC**：反直觉结果，可能源于MPC简化单轴模型与变形引起的模型失配
2. **前馈价值取决于轨迹激进程度**：温和轨迹改善小，激进轨迹改善15.9%
3. **LUT更新有效**：控制效率矩阵动态更新保障了变形过程中的基本稳定性
4. **IMU-ICD可行但仿真受限**：实机1kHz IMU响应真实，SITL中Gazebo IMU不够真实
5. **Stall Detection填补仿真空白**：SITL中位置/速度数据精确，堵转检测成为可靠的仿真验证手段

---

## 6. 待完成工作

### 6.1 实机实验（高优先级）

| 任务 | 说明 | 状态 |
|------|------|------|
| **固件编译与烧写** | `px4_fmu-v5_default` 编译成功 (88.73% FLASH)，已烧写到 Pixhawk 4 | ✅ **已完成** |
| **Airframe 4401 配置验证** | `SYS_AUTOSTART=4401`，所有参数（MPCA_MODE=2, PWM_MAIN_FUNC1=101, BAT1_N_CELLS=4 等）正确加载 | ✅ **已完成** |
| **变形机臂闭环控制** | `huaqiccc_morph_control` 模块开发完成：AS5600 I2C 读取 + P 控制 + AUX1 PWM 输出 + `huaqiccc_morph_angle` uORB 发布 | ✅ **已完成** |
| **V6C 同步编译** | `px4_fmu-v6c_default` 编译成功 (78.08% FLASH)，含 4401 airframe + morph 模块 | ✅ **已完成** |
| **室内动捕参数配置** | EKF2 (Vision定位, 无GPS, 无磁罗盘) + 着陆检测器松弛 + 安全保护配置 | ✅ **已完成** |
| 飞行控制性能测试 | 5种控制器实机圆轨迹飞行，记录位置误差 | ⏳ 待做 |
| 栖息成功率统计 | 160/200/250mm柱体，各≥10次重复 | ⏳ 待做 |
| 姿态稳定性记录 | 栖息后roll/pitch波动范围、振动幅值 | ⏳ 待做 |
| 功耗对比 | 悬停vs栖息状态的电机输出 | ⏳ 待做 |
| 振动检测验证 | 与参考加速度传感器对比 | ⏳ 待做 |
| 外场电线杆验证 | 混凝土电线杆栖息+振动采集 | ⏳ 待做 |
| 重新起飞（Deperch） | 展开→后退→悬停完整流程 | ⚠️ 代码待完善 |

### 6.2 代码完善（中优先级）

| 任务 | 说明 | 状态 |
|------|------|------|
| 角速度/角加速度前馈闭环 | _ang_vel_ff传递到mc_rate_control | ⚠️ 未实现 |
| 更激进轨迹测试 | R=2.0/T=10s验证前馈极限价值 | ⏳ 待做 |
| 风扰鲁棒性 | Gazebo wind plugin测试 | ⏳ 待做 |
| 惯量LUT实测标定 | 当前J LUT为线性插值，非实测值 | ⏳ 待做 |
| MPC参数系统性搜索 | alpha∈[5,15], r_delta∈[0.001,0.01] | ⏳ 待做 |
| **Stall Detection多轴支持** | 当前监控NED Y轴（对应Python X轴），实机可能从任意方向接近 | ⚠️ 待实现 |
| **电机饱和检测（Algo D）** | 0%假阳性，需集成电机反馈数据到mc_pos_control | ⚠️ 待评估 |

### 6.3 论文完善（中优先级）

| 任务 | 说明 | 状态 |
|------|------|------|
| 填充实机数据 | 表3轨迹对比、表4栖息成功率 | ⏳ 待实机数据 |
| 绘制FSM状态转移图 | 图3：5状态+转移条件 | ⏳ 待绘图 |
| 补充连续照片 | 栖息过程、外场实验 | ⏳ 待拍摄 |
| 摘要与结论 | 中英文摘要、引言 | ⏳ 待写 |
| 参考文献 | 补充近期相关文献 | ⏳ 待整理 |

---

## 7. 代码修改清单（新增）

### 7.1 新增模块

| 模块路径 | 功能 | 状态 |
|---------|------|------|
| `src/modules/huaqiccc_morph_control/` | **变形机臂闭环控制**：AS5600 I2C 读取 → P 控制 → AUX1 PWM 输出 → `huaqiccc_morph_angle` uORB 发布 | ✅ 已编译验证 |

### 7.2 修改的 PX4 核心文件

| 文件 | 修改内容 | 影响范围 |
|------|---------|---------|
| `src/modules/mavlink/mavlink_receiver.cpp` | `MAV_CMD_HUAQICCC_SET_ARM_ANGLE` 仅在 SITL (`CONFIG_ARCH_SIM`) 时直接发布 `huaqiccc_morph_angle`；实机仅 ACK | 避免 uORB topic 竞争 |
| `boards/px4/fmu-v5/default.px4board` | 启用 `CONFIG_MODULES_HUAQICCC_MORPH_CONTROL=y` | 构建系统 |
| `boards/px4/fmu-v6c/default.px4board` | 启用 `CONFIG_MODULES_HUAQICCC_MORPH_CONTROL=y` | 构建系统 |
| `ROMFS/px4fmu_common/init.d/airframes/4401_huaqiccc_real` | 新增 morph 参数（MORPH_EN, MORPH_KP, MORPH_DB, PWM_AUX_MIN1/MAX1 等）和启动命令 | 实机配置 |

---

## 8. 实机配置记录（4401_huaqiccc_real）

### 7.1 硬件清单

| 组件 | 型号 | 连接 |
|------|------|------|
| 飞控 | Pixhawk 4 (FMU-v5) | USB `/dev/ttyACM0` |
| 电机 | T-Motor MN3508 KV380 | MAIN1-4 (PWM) |
| 电调 | 好盈 40A | 与电机配套 |
| 电池 | 4S LiPo | Pixhawk 4 Power Module |
| GPS | u-blox NEO-M8N | GPS1 端口 |
| 数传 | 915MHz | TELEM1 |
| RC | 未配置 | 预留 |

### 7.2 关键参数摘要（已验证）

```
SYS_AUTOSTART      = 4401      # huaqiccc Real Hardware
MPCA_MODE          = 2         # LQR 控制器
MPCA_FF_MASS       = 1.173     # 论文质量 1173g
MPCA_FF_EN         = 1         # Flatness feedforward 启用
EFO_ENABLE         = 1         # 外力估计启用
MPCA_PC_EN         = 0         # Perching 默认禁用（安全）
MPC_XY_P           = 1.5       # 保守位置增益
MPC_THR_HOVER      = 0.55      # 悬停推力估计
PWM_MAIN_FUNC1     = 101       # Motor1 → MAIN1
PWM_MAIN_FUNC2     = 102       # Motor2 → MAIN2
PWM_MAIN_FUNC3     = 103       # Motor3 → MAIN3
PWM_MAIN_FUNC4     = 104       # Motor4 → MAIN4
BAT1_N_CELLS       = 4         # 4S LiPo
CBRK_IO_SAFETY     = 22027     # 禁用安全开关（⚠️ 仅台架测试）

# Morphing Arm Control
MORPH_EN           = 1         # 变形控制模块启用
MORPH_KP           = 2.0       # P 增益
MORPH_DB           = 0.02      # 死区（归一化）
MORPH_EMIN         = 0         # 编码器 @ 闭合 (需标定)
MORPH_EMAX         = 4095      # 编码器 @ 展开 (需标定)
MORPH_RATE         = 50        # 控制频率 (Hz)
MORPH_BUS          = 2         # I2C bus (Pixhawk 4 I2C A)
PWM_AUX_MIN1       = 1050      # 推杆缩短 PWM (μs)
PWM_AUX_MAX1       = 1950      # 推杆伸长 PWM (μs)
PWM_AUX_DIS1       = 1500      # 推杆停止 PWM (μs)
```

### 7.3 烧写命令

```bash
cd ~/PX4-Autopilot
make px4_fmu-v5_default -j$(nproc)
python3 Tools/px_uploader.py --port /dev/ttyACM0 build/px4_fmu-v5_default/px4_fmu-v5_default.px4
# 首次切换 airframe 后需手动设置：
param set SYS_AUTOSTART 4401
param save
reboot
```

> ⚠️ **安全提醒**：`CBRK_IO_SAFETY=22027` 禁用安全开关，电机可在无物理开关解锁情况下启动。**正式飞行前必须改回 `CBRK_IO_SAFETY=0`**。

---

## 8. 环境信息

| 组件 | 版本/配置 |
|------|----------|
| OS | Ubuntu 20.04 |
| ROS | Noetic (desktop-full) |
| Gazebo | Classic 11 |
| PX4 | v1.14.3 + huaqiccc modifications |
| MAVROS | 1.16.0+ |
| Python | 3.8+ (pandas, numpy) |
| 编译器 | GCC 9.4+ |
| 硬件目标 | **Pixhawk FMU-v5** (FLASH 88.73%，实机已烧写验证) |
| SITL模型 | huaqiccc (1.5 kg, arm_angle∈[0,-0.3]rad) |

### 关键路径

```
PX4源码: ~/PX4-Autopilot/
测试脚本: ~/huaqiccc_test_suite/flight/*.py, ~/huaqiccc_test_suite/perching/*.py
运行脚本: ~/run_huaqiccc_test.sh, ~/huaqiccc_test_suite/runners/*.sh
日志输出: ~/huaqiccc_logs/*.csv
ULOG: ~/.ros/log/YYYY-MM-DD/*.ulg
论文大纲: ~/PX4-Autopilot/大纲0418/
论文草稿: ~/PX4-Autopilot/大纲0418/章节草稿/
```

---

*文档生成时间：2026-06-04*
*版本：v2.4（新增室内动捕参数配置、EKF2 Vision定位、着陆检测器松弛）*
EOF
echo "进度总结已生成：$(wc -l < /home/a/PX4-Autopilot/回顾总结_20260527/PROJECT_PROGRESS_SUMMARY.md) 行"