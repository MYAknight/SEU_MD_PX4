# Huaqiccc 变形四旋翼 — 当前状态（2026-06-14）

> 本文档记录固件代码层面的最新修改和待验证事项。  
> 固件源码：`~/Projects/PX4/SEU_MD_PX4` (PX4 v1.14.3)

---

## 1. 今日完成的代码修改（2026-06-11）

### 1.1 LUT 表修复
**文件**：`src/modules/control_allocator/huaqiccc_motor_lut.hpp`

| 修改项 | 旧值 | 新值 |
|--------|------|------|
| motor 编号顺序 | `0=lb,1=lf,2=rb,3=rf` | **`0=rb,1=rf,2=lb,3=lf`** |
| index=0 (收拢状态) | (-0.188,+0.249) 等 | **(-0.155,+0.215)** 等（实机测量值基准）|
| PZ 值 | -0.03842 | **0.0** |
| `HUAQICCC_LUT_MAX_ANGLE` | -0.50 | **-0.40** |

**关键说明**：motor 顺序必须与 `ControlAllocator.cpp` 的列顺序严格一致。

### 1.2 ControlAllocator 修复
**文件**：`src/modules/control_allocator/ControlAllocator.cpp`

```cpp
// 旧
const float km[4] = {-0.05f, 0.05f, 0.05f, -0.05f};
// 新 — 匹配 airframe CA_ROTOR_KM
const float km[4] = {-0.06f, +0.06f, +0.06f, -0.06f};
```

顺序：`rb=CW(-), rf=CCW(+), lb=CCW(+), lf=CW(-)`

### 1.3 实机 Airframe 更新
**文件**：`ROMFS/px4fmu_common/init.d/airframes/4401_huaqiccc_real`

按实机测量值与历史成功飞行配置更新：
- `COM_RC_IN_MODE=3`（RC + Joystick 双模式）
- 电机几何：`CA_ROTOR0=(-0.155,+0.215)`, `CA_ROTOR1=(+0.205,+0.165)` 等
- `CA_ROTOR_KM`：`±0.06`（匹配电机实际转向）
- `PWM_MAIN_MIN/MAX=1100/1900`（ESC 行程限制）
- `CA_METHOD=2`
- `MPC_THR_HOVER=0.45`
- `MPCA_FF_MASS=1.15`
- `MPCA_MODE=0`（首飞保守，原始 PID）
- `MORPH_EN=1`（默认启用变形，但首次验证需确认 AS5600）
- 启动命令：`huaqiccc_morph_control start -b 2`

### 1.4 SITL Airframe 同步
**文件**：`ROMFS/px4fmu_common/init.d-posix/airframes/4400_gazebo-classic_huaqiccc`

电机参数已同步为与实机一致（收拢状态）。仿真用旧 SDF 物理位置，但编号/转向已对应。

### 1.5 SDF 模型修复
**文件**：`Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/huaqiccc/huaqiccc.sdf`

motor plugin 重新映射：
| Plugin | motorNumber | turningDirection |
|--------|-------------|------------------|
| rbmotor | 0 | cw |
| rfmotor | 1 | ccw |
| lbmotor | 2 | ccw |
| lfmotor | 3 | cw |

### 1.6 删除错误旧机型
**删除的文件**：`ROMFS/px4fmu_common/init.d/airframes/4400_huaqiccc`

这是一个错误放置在实机 ROMFS 目录中的旧版 SITL airframe，参数完全错误：
- 电机坐标是旧设计值（-0.046568, 0.248013 等）
- 所有 `KM=0.06`（没有 CW/CCW 区分）
- `CA_METHOD=1`
- `MPC_THR_HOVER=0.55`
- 无 PWM 限制、无 EKF2 MoCap 配置、无 morph 控制

**同时从 `CMakeLists.txt` 构建列表中移除**，确保不再打包进固件。

### 1.7 变形控制参数清理
**文件**：`src/modules/huaqiccc_morph_control/huaqiccc_morph_control_params.c`、`HuaqicccMorphControl.hpp`

- 删除未使用的 `MORPH_KP` 和 `MORPH_DB` 参数
- 原因：`HuaqicccMorphControl.cpp` 的 Bang-Bang 闭环中未读取这两个参数，死区硬编码为 `DEADBAND_RAD = 0.01f`
- 保留参数：`MORPH_EN`, `MORPH_EMIN`, `MORPH_EMAX`, `MORPH_RATE`, `MORPH_BUS`

### 1.8 注释一致性修复
**文件**：`src/modules/control_allocator/huaqiccc_motor_lut.hpp`、`src/modules/huaqiccc_morph_control/huaqiccc_morph_control_params.c`、`HuaqicccMorphControl.hpp`

- 统一 `arm_angle` 机械限位描述为 **-0.40 rad**
- 修正 LUT motor 顺序注释 `0=rb, 1=rf, 2=lb, 3=lf`

### 1.9 接触/栖停遥测通道重构（2026-06-14）
**文件**：`src/modules/mc_pos_control/MulticopterPositionControl.cpp/.hpp`

- 新增 `debug_array` uORB 发布 `_perch_debug_pub`
- 每个控制周期发布 `DEBUG_FLOAT_ARRAY`（`name="perch"`），包含：

| `data[i]` | 含义 | 数值 |
|---|---|---|
| `data[0]` | 接触检测状态 | 0=IDLE, 1=CANDIDATE, 2=DETECTED |
| `data[1]` | 栖停阶段 | 0=NONE, 1=APPROACH, 2=CONTACT, 3=COMPLIANT, 4=RAMP_DOWN, 5=PERCHED |
| `data[2]` | `MPCA_PC_EN` | 0/1/2/3/4 |
| `data[3]` | 变形臂角度（rad） | 无数据时为 99.0 |
| `data[4]` | 抓握确认 | 0/1 |
| `data[5]` | 栖停激活 | 0/1 |

- 地面站通过 `/mavros/debug_value/debug_float_array` 接收并解析，避免依赖 `STATUSTEXT`

**实机验证**：
- ✅ V6C 固件编译通过（FLASH 77.87%）
- ✅ 已刷入 Pixhawk 6C
- ✅ 手动栖停测试：新检测器灵敏度合适，遥测数据稳定到达地面站

### 1.10 自主栖停 FSM 逻辑优化（2026-06-16）

**文件**：
- `src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- `src/modules/mc_pos_control/MulticopterPositionControl.hpp`
- `src/modules/mc_pos_control/mc_pos_control_params.c`
- `~/Projects/ground_station/scripts/control_ground_station_ros.py`

**关键改动**：

| 改动项 | 旧行为 | 新行为 |
|---|---|---|
| COMPLIANT 位置 P 增益 | X/Y/Z 全乘 `MPCA_PC_K_SOFT` | **仅 X/Y 乘软化系数**，Z 保持正常 |
| COMPLIANT 积分 | 进入时重置全部积分 | **不再重置积分**，保留 Z 轴积分以维持高度 |
| 推力模型 | 默认 `MPCA_PC_SPRING=1`，弹簧模型上限 1.3×hover | 默认 **`MPCA_PC_SPRING=0`**，由 Z 位置控制器直接维持高度；弹簧上限放宽到 2.0×hover |
| 抓握判定 | `pos_stable && (arms_contracted \|\| elapsed>6s)` 时间兜底 | **必须 `pos_stable && arms_contracted`**，无时间兜底 |
| 进入 RAMP_DOWN | `elapsed>6s && grasp_secure` | **`grasp_secure` 后保持 1s** 即进入缓降 |
| setpoint override | 阈值写死 `>=3`（永远不满足） | 修正为 **`>=2`**，COMPLIANT 使用固定 `_compliant_sp_x` 不随动 |
| 机械臂收拢 | 需手动/外部触发 | 地面站检测到 `CONTACT/COMPLIANT` 后**自动下发一次 `MAV_CMD_HUAQICCC_SET_ARM_ANGLE(0.0)`** |
| 收拢阈值 | 硬编码 `-0.30 rad` | 新增参数 **`MPCA_PC_ARM_THR=-0.15 rad** |

**新增/调整参数**：
- `MPCA_PC_K_SOFT`：默认值 `0.20` → `0.50`
- `MPCA_PC_SPRING`：默认值 `1` → `0`
- `MPCA_PC_ARM_THR`：新增，默认 `-0.15`

**验证状态**：
- ✅ `px4_fmu-v6c_default` 编译通过（FLASH 77.87%）
- ✅ `px4_sitl_default` 编译通过
- ⏳ 待实机飞行验证

---

## 2. 编译状态

| 目标 | 状态 | 说明 |
|------|------|------|
| `px4_sitl_default` | ✅ 通过 | 18/18 目标 |
| `px4_fmu-v6c_default` | ✅ 通过 | 890/890 目标，clean build |

**固件输出**：
```
build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

---

## 3. 刷写与验证（已完成 ✅）

### 3.1 刷写步骤（已完成）
1. ✅ QGC → Vehicle Setup → Firmware → 选择"自定义固件文件"
2. ✅ 选择：`build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`
3. ✅ 等待刷写完成，并执行物理断电重启

### 3.2 刷写后验证清单（全部通过）

```bash
# 1. 确认固件版本和时间
ver all
# 预期：Build datetime: Jun 11 2026 18:03:xx

# 2. 确认只有一个正确机型
# QGC → Airframe → Quadrotor with morphing arms
# 应该只显示：huaqiccc Morphing Quadrotor (Real Hardware) [4401]

# 3. 确认 MORPH_EN 参数存在
param show MORPH_EN
# 预期：MORPH_EN = 1 (default)

# 4. 确认所有 Morph 参数
param show | grep MORPH
# 预期：MORPH_EN, MORPH_EMIN, MORPH_EMAX, MORPH_RATE, MORPH_BUS
# 注：MORPH_KP / MORPH_DB 已删除，代码中使用固定 Bang-Bang 死区

# 5. 确认 morph 模块启动
dmesg | grep huaqiccc
# 预期：huaqiccc_morph_control started on I2C bus 2, AS5600 raw=...
# 如果 MORPH_EN=1 但没有这条日志，说明 AS5600 未响应

# 6. 确认加载了正确的 airframe
# dmesg 中应有：Loading airframe: /etc/init.d/airframes/4401_huaqiccc_real

# 7. 确认 CA_ROTOR 参数正确
param show CA_ROTOR0_PX  # 预期：-0.155
param show CA_ROTOR1_PX  # 预期：+0.205
param show CA_ROTOR0_KM  # 预期：-0.06
param show CA_ROTOR1_KM  # 预期：+0.06
```

---

## 4. 关键发现与注意事项

### 4.1 之前飞行正常的真正原因（历史记录）
- **实机参数 `MORPH_EN=0`**（变形禁用）
- → `huaqiccc_morph_control` 模块不启动
- → 不发布 `huaqiccc_morph_angle`
- → `ControlAllocator` 的 `huaqiccc_active` 始终为 false
- → **走标准路径（使用 airframe 的 CA_ROTOR 参数），LUT 完全不被调用**
- → 因此即使 LUT 有错误，也**不影响飞行**
- **当前状态**：已刷写 6月11日固件，`MORPH_EN=1`，LUT 已在实机飞行中验证正常工作。

### 4.2 启用变形后的验证结果
- `MORPH_EN=1` 后模块正常启动，50Hz 发布 `huaqiccc_morph_angle`
- LUT 被调用，effectiveness matrix 在飞行中实时更新
- **实机验证**：变形功能在飞行中可靠，四种算法（PID/GS-PID/LQR/MPC）均稳定

### 4.3 AS5600 状态
- ✅ AS5600 在 `MORPH_EN=1` 时正常响应
- ✅ `dmesg | grep huaqiccc` 可看到模块启动日志
- ✅ 飞行中 morph 角度通过 `DEBUG_FLOAT_ARRAY`（`perch.data[3]`）正常回传地面站
  - ✅ 接触/栖停状态通过 `DEBUG_FLOAT_ARRAY`（`perch.data[0/1]`）正常回传地面站

### 4.4 SITL vs 实机坐标差异
- SDF 中电机物理位置仍沿用旧模型坐标
- 仅调整了编号/转向顺序，与 CA_ROTOR 顺序对应
- 用户接受"仿真只验证顺序，不追求完全一致的质心位置"

---

## 5. 电机 ↔ CA_ROTOR ↔ LUT 映射（最终确认）

| 物理电机 | 电调输出 | CA_ROTOR | LUT motor | 转向 |
|---------|---------|----------|-----------|------|
| motor1 (rb) | MAIN4 | CA_ROTOR0 | LUT motor0 | CW (KM=-0.06) |
| motor2 (rf) | MAIN3 | CA_ROTOR1 | LUT motor1 | CCW (KM=+0.06) |
| motor3 (lb) | MAIN2 | CA_ROTOR2 | LUT motor2 | CCW (KM=+0.06) |
| motor4 (lf) | MAIN1 | CA_ROTOR3 | LUT motor3 | CW (KM=-0.06) |

---

## 6. 备份位置

修改前的原始文件备份在：
```
~/Desktop/backup_lut/
├── huaqiccc_motor_lut.hpp       # 原始 LUT（motor 顺序错误）
├── ControlAllocator.cpp         # 原始 ControlAllocator（km 值旧）
└── 4400_gazebo-classic_huaqiccc.bak  # 原始 SITL airframe
```

---

## 7. 已完成验证清单（2026-06-11）

| 优先级 | 任务 | 状态 | 说明 |
|--------|------|------|------|
| 🔴 P0 | 刷写最新固件 | ✅ | 6月11日固件已刷入，物理断电重启正常 |
| 🔴 P0 | 验证 QGC 机型列表 | ✅ | 仅显示 4401_huaqiccc_real，无 4400 |
| 🔴 P0 | 验证 MORPH_EN 参数 | ✅ | `param show MORPH_EN` 显示为 1 |
| 🟡 P1 | 验证 AS5600 硬件 | ✅ | 模块启动日志正常，编码器反馈正常 |
| 🟡 P1 | 首飞测试 | ✅ | 位置模式 / OFFBOARD 正常起飞 |
| 🟢 P2 | 变形飞行验证 | ✅ | 机臂展开/收拢可靠，LUT 实时更新正常 |
| 🟢 P2 | OFFBOARD 轨迹跟踪 | ✅ | 地面站发送 setpoint，跟踪正常 |
| 🟢 P2 | 四种算法验证 | ✅ | PID / GS-PID / LQR / MPC 实机控制均可靠 |
| 🟢 P2 | 位置/姿态接触检测 | ✅ | 实机手动栖停验证，灵敏度合适，遥测链路正常 |

## 8. 后续待办

| 优先级 | 任务 | 说明 |
|--------|------|------|
| 🟢 P2 | 视觉 YAW 对齐实机集成 | YOLO + OFFBOARD 协调 |
| 🟢 P2 | 一键栖停 | 完整 GUI 集成 |
| 🟢 P2 | 长航时变形耐久性 | 多次变形循环的可靠性 |
| 🟢 P2 | 室外 GPS 测试 | M10 GPS 外场验证 |
