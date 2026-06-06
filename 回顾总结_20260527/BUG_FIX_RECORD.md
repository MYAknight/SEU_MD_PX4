# Bug修复记录

> 本文件记录项目开发过程中的关键Bug及其修复方案。
> 按严重程度和修复时间排序。

---

## 致命级Bug（会导致系统失控或崩溃）

### BUG-001：MAVROS加速度前馈未传递

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-19 |
| **严重级别** | 🔴 致命 |
| **影响模块** | mavlink_receiver, FlatnessFeedforward, MPC+FF |
| **现象** | Python测试脚本发送的加速度前馈（afx/afy/afz）未到达PX4控制器，微分平坦前馈完全失效 |
| **根因** | `SET_POSITION_TARGET_LOCAL_NED`消息处理代码仅将afx/afy/afz读取到局部变量，但未复制到`setpoint.acceleration`数组 |

**修复代码**（`src/modules/mavlink/mavlink_receiver.cpp`，约第997行）：
```cpp
// 修复前：afx/afy/afz仅保存在局部变量，未传递给setpoint
float afx = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? NAN : target_local_ned.afx;
float afy = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? NAN : target_local_ned.afy;
float afz = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? NAN : target_local_ned.afz;
// setpoint.acceleration未被赋值！

// 修复后：
setpoint.acceleration[0] = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? (float)NAN : target_local_ned.afx;
setpoint.acceleration[1] = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? (float)NAN : target_local_ned.afy;
setpoint.acceleration[2] = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? (float)NAN : target_local_ned.afz;
```

**验证方法**：启用MPC+FF模式，检查`_flat_output_valid`是否为true，以及`_acc_sp_ff`是否有非零值。

---

### BUG-002：单位混淆导致推力失控

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-19 |
| **严重级别** | 🔴 致命 |
| **影响模块** | AdvancedPositionControl, FlatnessFeedforward |
| **现象** | 启用flatness feedforward后，无人机瞬间失控、姿态翻转 |
| **根因** | 直接将flatness输出的`collective_thrust`（单位：N，牛顿）叠加到`_thr_sp`（单位：归一化推力，0-1），导致量纲错误。15N被当作15倍最大推力 |

**错误代码**（已删除）：
```cpp
// 错误：N与归一化推力直接相加
_thr_sp += ff.body_z * ff.collective_thrust * _flatness_blend;
```

**修复代码**（`src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.cpp`）：
```cpp
// 正确：将flatness加速度[m/s²]注入_acc_sp
if (_flatness_ff.compute(_flat_output, _vehicle_mass, J, 0.0f, ff)) {
    _acc_sp_ff = Vector3f(_flat_output.acc) * _flatness_blend;
    _ang_vel_ff = ff.angular_velocity * _flatness_blend;
    _ang_acc_ff = ff.angular_acceleration * _flatness_blend;
}
// ... MPC计算后 ...
ControlMath::addIfNotNanVector3f(_acc_sp, _acc_sp_ff);
_accelerationControl();  // _acc_sp → _thr_sp（单位转换在此完成）
```

**关键原理**：`_acc_sp`的单位是m/s²（加速度），`_thr_sp`的单位是归一化推力（0-1）。`_accelerationControl()`内部通过`hover_thrust`和重力加速度完成单位转换。前馈必须以加速度形式注入，不能绕过该函数直接修改推力。

**验证方法**：检查MPC+FF模式下，悬停时_thr_sp约为-hover_thrust，而非巨大负数。

---

### BUG-003：参数未持久化导致测试不一致

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-20 |
| **严重级别** | 🟡 高 |
| **影响模块** | 测试脚本, ROMFS参数系统 |
| **现象** | 每次测试后，MPCA_MODE等参数被保存，下一次测试即使修改ROMFS默认值，PX4仍读取上一次的保存值 |
| **根因** | PX4优先读取`build/px4_sitl_default/rootfs/parameters.bson`而非ROMFS默认值 |

**修复方案**（`~/run_flatness_test.sh`）：
```bash
# 在启动PX4前删除保存的参数文件
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters_backup.bson
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/parameters.bson
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/parameters_backup.bson
```

**验证方法**：连续两次运行不同MPCA_MODE的测试，确认第二次测试使用了正确的模式。

---

### BUG-004：MAVROS端口不匹配

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-18 |
| **严重级别** | 🟡 高 |
| **影响模块** | MAVROS连接 |
| **现象** | MAVROS无法连接PX4 SITL，roslaunch启动后无MAVLink消息 |
| **根因** | `mavros_posix_sitl.launch`中`fcu_url`设置为`udp://:14540@localhost:14557`，但PX4 SITL实际监听端口为14580 |

**修复**：
```xml
<!-- 修复前 -->
<arg name="fcu_url" default="udp://:14540@localhost:14557"/>
<!-- 修复后 -->
<arg name="fcu_url" default="udp://:14540@localhost:14580"/>
```

---

### BUG-005：Environment变量未export导致参数不生效

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-20 |
| **严重级别** | 🟡 高 |
| **影响模块** | 测试脚本参数传递 |
| **现象** | Python脚本无法读取bash中设置的MPCA_MODE和MPCA_FF_EN |
| **根因** | `run_flatness_test.sh`中仅`MPCA_MODE=${1:-3}`未加`export`，子进程不可见 |

**修复**：
```bash
# 修复前
MPCA_MODE=${1:-3}
MPCA_FF_EN=${2:-1}

# 修复后
export MPCA_MODE=${1:-3}
export MPCA_FF_EN=${2:-1}
```

---

## 中等级Bug（影响功能但不致命）

### BUG-006：测试脚本提前landing（total_time=52s）

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-26 |
| **严重级别** | 🟢 中 |
| **影响模块** | huaqiccc_flatness_test.py |
| **现象** | 测试脚本仅运行52s就landing，没有hover1阶段 |
| **根因** | TrajectoryGenerator.total_time = 10+8+20+8+3+3 = 52s，这是设计值，但之前记录中expect了100s的测试 |
| **状态** | 非Bug，是脚本设计如此。total_time=52s包含完整流程 |

**说明**：如果需要更长的hover验证，需修改`T_HOVER1`或添加额外hover阶段。

---

### BUG-007：LQR LUT使用静态初始值

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-19 |
| **严重级别** | 🟢 中 |
| **影响模块** | AdvancedPositionControl, LQR模式 |
| **现象** | LQR增益查找表仅index=0有有效值，其余为0 |
| **根因** | LQR LUT初始化时只填充了第一个元素 |
| **状态** | 已修复，LQR LUT在代码中已填充所有采样点的增益值 |

---

### BUG-008：测试套件入口脚本 pkill 自杀

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-27 |
| **严重级别** | 🟡 高 |
| **影响模块** | huaqiccc_test_suite runners |
| **现象** | 运行 `run_huaqiccc_test.sh` 或任意 runner 脚本时，进程在启动阶段被 kill -9 终止 |
| **根因** | `pkill -9 -f "huaqiccc"` 会匹配到脚本自身路径（如 `~/huaqiccc_test_suite/runners/01_flatness_circle.sh`），导致自杀 |

**修复**：将 runners 下所有脚本中的 pkill 模式去掉 `huaqiccc`，仅保留 `roslaunch|roscore|gzserver|gzclient|px4|mavros_node`。

```bash
# 修复前
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node|huaqiccc"
# 修复后
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node"
```

### BUG-009：mavros_posix_sitl.launch pxh> 刷屏

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-05-27 |
| **严重级别** | 🟢 中 |
| **影响模块** | SITL 启动日志 |
| **现象** | 通过 `mavros_posix_sitl.launch` 启动 SITL 时，终端被 `pxh>` 刷屏 |
| **根因** | `mavros_posix_sitl.launch` 中 `interactive` 默认值为 `true`，而 `posix_sitl.launch` 已改为 `false` |

**修复**：
```xml
<!-- 修复前 -->
<arg name="interactive" default="true"/>
<!-- 修复后 -->
<arg name="interactive" default="false"/>
```

---

### BUG-010：`pc_trig` 门控逻辑错误（Stall Detection 与 IMU Contact 同时触发/同时禁用）

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-06-01 |
| **严重级别** | 🔴 致命 |
| **影响模块** | `mc_pos_control` perching FSM |
| **现象** | `MPCA_PC_TRIG=1`（仅Stall）时IMU路径仍运行；`MPCA_PC_TRIG=2`（仅IMU）时Stall路径仍运行 |
| **根因** | 两个触发门都错误地使用了 `pc_trig != 1`，导致 `MPCA_PC_TRIG` 参数完全失效 |

**修复代码**（`src/modules/mc_pos_control/MulticopterPositionControl.cpp`）：
```cpp
// 修复前（错误）：两个门都用 != 1
if (pc_en >= 2 && pc_trig != 1 && ...) {  // IMU路径
if (pc_en >= 2 && pc_trig != 1 && ...) {  // Stall路径（错误！）

// 修复后（正确）：
if (pc_en >= 2 && pc_trig != 1 && ...) {  // IMU路径：禁用当 STALL_ONLY(1)
if (pc_en >= 2 && pc_trig != 2 && ...) {  // Stall路径：禁用当 IMU_ONLY(2)
```

**验证方法**：设置 `MPCA_PC_TRIG=1`，确认只有 `STALL_DETECTED` 触发；设置 `MPCA_PC_TRIG=2`，确认只有 IMU contact 触发。

---

### BUG-011：Stall Detection 坐标轴不匹配（X轴监控 vs Y轴飞行轨迹）

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-06-01 |
| **严重级别** | 🔴 致命 |
| **影响模块** | `mc_pos_control` Stall Detection |
| **现象** | SITL中无人机飞向杆子，但Stall Detection监控X轴，y_error≈0，approaching永远为false，STALL永不触发 |
| **根因** | MAVROS将Python ENU坐标转换为PX4 NED坐标：Python X轴 → NED Y轴。代码错误使用`position[0]`（X轴）监控，而实际飞行轨迹在NED Y轴 |

**修复代码**：
```cpp
// 修复前（错误）：监控X轴
float x_error = _setpoint.position[0] - states.position(0);
float x_vel = states.velocity(0);

// 修复后（正确）：监控Y轴（与NED飞行轨迹一致）
float y_error = _setpoint.position[1] - states.position(1);
float y_vel = states.velocity(1);
```

**新增参数**（支持动态阈值）：
```cpp
bool approaching = y_error > _param_mpca_pc_stall_err.get();    // MPCA_PC_SERR=0.05
bool nearly_stopped = fabsf(y_vel) < _param_mpca_pc_stall_vel.get();  // MPCA_PC_SVEL=0.10
```

> ⚠️ **易错点**：MAVROS的`setpoint_raw/local`话题会强制将ROS ENU坐标转换为PX4 NED坐标（无论`coordinate_frame`如何设置）。Python脚本中的`x`对应PX4 NED的`y`，`y`对应PX4 NED的`x`。所有PX4内部控制逻辑必须使用NED坐标系判断。

---

### BUG-012：Stall Detection 日志在 perching 后持续刷屏

| 属性 | 内容 |
|------|------|
| **发现时间** | 2026-06-01 |
| **严重级别** | 🟢 中 |
| **影响模块** | `mc_pos_control` 日志系统 |
| **现象** | 进入 CONTACT/COMPLIANT 后，`STALL_DETECTED` 每秒打印数十次，日志文件膨胀 |
| **根因** | Stall Detection 代码每轮控制循环都执行，mavlink_log_info 未检查当前 perching 状态 |

**修复代码**：
```cpp
if (_stall_start_time == 0) {
    _stall_start_time = hrt_absolute_time();
    _stall_start_x = states.position(0);
    if (_perching_phase == PerchingPhase::NONE) {
        mavlink_log_info(&_mavlink_log_pub, "STALL_START");
    }
} else {
    // ... stall_detected = true ...
    if (_perching_phase == PerchingPhase::NONE) {
        mavlink_log_info(&_mavlink_log_pub, "STALL_DETECTED");
    }
}
```

---

## 待调查问题

| 问题 | 描述 | 优先级 |
|------|------|--------|
| LQR hover误差偏高 | LQR hover XY=0.1166m，略高于其他算法(0.10m) | 低 |
| MPC性能低于预期 | MPC误差(0.126m)显著差于LQR(0.074m)，需分析模型失配 | 中 |
| 前馈角速度未闭环 | _ang_vel_ff计算但未传递到mc_rate_control | 中 |

---

*记录时间：2026-06-01*
*版本：v1.2（新增BUG-010/011/012，Stall Detection修复）*
EOF
echo "Bug修复记录已生成"