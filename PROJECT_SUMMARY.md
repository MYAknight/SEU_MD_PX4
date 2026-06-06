# Huaqiccc 变形四旋翼 PX4 项目总结文档

> 本文档记录了从原始 PX4 v1.14.3 到当前精简版的所有修改，用于在新对话中快速恢复上下文。
> 生成时间：2026-05-19

---

## 1. 项目目标

### 1.1 核心目标
开发 **huaqiccc 变形四旋翼无人机**，通过机械臂角度变化改变电机位置，实现可变形飞行。

### 1.2 关键技术点
- **机型**：Morphing Quadrotor（四旋翼，机臂可变形）
- **FCU**：Pixhawk V6C (`px4_fmu-v6c_default`)
- **仿真**：Gazebo Classic SITL
- **控制接口**：MAVLink 31440 自定义命令 → uORB `huaqiccc_morph_angle`
- **定位**：室内动捕 / 室外 GPS (M10 GPS，UART 接口)
- **通信**：MAVLink + MAVROS (ROS1 Noetic)，**不使用 ROS2**
- **日志**：需要板载 ulog 日志

### 1.3 变形机制
- 运行时通过 LUT (Lookup Table) 根据 `arm_angle` 查询电机位置 (px, py) 和力臂
- 实时重建控制分配 effectiveness 矩阵
- SITL 使用 `{-0.05, +0.05, +0.05, -0.05}` KM 符号；实机使用 LUT 覆盖

---

## 2. 项目结构

```
/home/a/PX4-Autopilot/          # 精简后的 PX4 源码
/home/a/catkin_ws/              # ROS1 工作空间
/home/a/huaqiccc_simplified_flight_test.py   # 飞行测试脚本
/home/a/run_simplified_test.sh  # 一键飞行测试 bash 脚本
/home/a/huaqiccc_logs/          # 飞行数据 CSV
~/huaqiccc_backup_v2_20260519/  # 完整备份（含 restore.sh）
~/PX4-Autopilot_pruned_backup_20260519_053000/  # 删除文件的备份
```

---

## 3. 已完成的修改（按阶段）

### Phase 0: 基础修复（Before Pruning）
- **Crash Fix**：修复了 `px4-rc.params` 使用不存在的参数名 `MPC_XY_VEL_P`/`I`（正确应为 `MPC_XY_VEL_P_ACC`/`I_ACC`）
- **SITL Airframe 恢复**：从备份恢复了 `4400_gazebo-classic_huaqiccc`，修正 KM 符号匹配 SDF
- **MPC 调参**：最终参数 `MPC_XY_P=3.8`, `MPC_XY_TRAJ_P=0.7`, `MPC_XY_VEL_P_ACC=2.2`
- **Smooth Trajectory**：使用 smoothstep 实现 C1 连续轨迹，消除相位边界突变
- **V6C 固件编译成功**：添加 `@class Copter`，修复 NuttX 兼容性（移除 `<string>`）

### Phase 1: ROMFS 精简
- 删除了 `init.d/airframes/` 中 24 个非 MC airframes（plane, VTOL, heli, rover, UUV, airship）
- 删除了 `init.d-posix/airframes/` 中 25 个非 MC SITL airframes
- 修复了 `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` 和 `init.d-posix/airframes/CMakeLists.txt`
- 恢复了 `airspeed` 库（sensors 模块依赖）
- 添加了 `4400_gazebo-classic_huaqiccc` 到 `init.d-posix/airframes/CMakeLists.txt`

### Phase 2: 非 MC 模块禁用
- **SITL board config**：禁用 `AIRSHIP_ATT_CONTROL`, `AIRSPEED_SELECTOR`, `FW_*, ROVER_*, UUV_*, VTOL_*`
- **V6C board config**：禁用同样的非 MC 模块
- **删除 land_detector 非 MC 类**：Fixedwing, VTOL, Rover, Airship
- **删除 control_allocator 非 MC effectiveness**：FixedWing, Helicopter, VTOL x3, Rover x2, UUV, ControlSurfaces
- **删除 weather_vane 库** 及其在 flight_mode_manager 中的引用
- **删除 ADSB mavlink streams** 和 navigator 中的 ADSB 代码

### Phase 3: Commander 深入精简
- **删除 airspeedCheck / vtolCheck**（HealthAndArmingChecks）
- **简化 commander_helper**：`is_vtol→false`, `is_fixed_wing→false`, `is_ground_vehicle→false`
- **删除 failsafe.cpp** 中 VTOL/fixedwing 逻辑和 quadchute 检查
- **删除 mode_requirements.cpp** fixedwing 要求
- **删除 FailureDetector.cpp** tailsitter 90° 姿态处理
- **简化 Commander.cpp**：vehicle_type 始终设为 ROTARY_WING；vtolStatusUpdate() 为空；简化 orbit 处理

### Phase 4: 传感器驱动精简（本轮）
- **V6C board config 禁用**：
  - `UAVCAN` (无 CAN 设备)
  - `uXRCE_DDS_CLIENT` (不用 ROS2)
  - `CAMERA_TRIGGER/CAPTURE/FEEDBACK` (无相机)
  - `GIMBAL` (无云台)
  - `LANDING_TARGET_ESTIMATOR` (不用精准降落)
- **禁用 COMMON 宏**：
  - `CONFIG_COMMON_DIFFERENTIAL_PRESSURE=n` (不用空速)
  - `CONFIG_COMMON_OPTICAL_FLOW=n` (不用光流)
  - `CONFIG_COMMON_TELEMETRY=n` (不用遥测)
  - `CONFIG_COMMON_MAGNETOMETER=n` + 单独启用 `IST8310`
- **结果**：多余驱动全部移除，磁力计从 12 个减至 1 个

---

## 4. 关键文件清单

### 4.1 变形核心代码
| 文件 | 作用 |
|------|------|
| `src/modules/control_allocator/ControlAllocator.cpp` | 拦截 `huaqiccc_morph_angle` uORB，LUT 查表重建 effectiveness 矩阵 |
| `src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp` | 标准 rotor effectiveness，使用 `_param_ca_rotor_count` |
| `src/modules/mavlink/mavlink_receiver.cpp` | 处理 MAVLink 31440 命令，发布到 uORB |
| `ROMFS/px4fmu_common/init.d-posix/px4-rc.params` | SITL 启动参数注入 (MPC 调参) |
| `ROMFS/px4fmu_common/init.d/airframes/4400_huaqiccc` | 实机 airframe，含 `@class Copter` |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4400_gazebo-classic_huaqiccc` | SITL airframe |

### 4.2 飞行测试代码
| 文件 | 作用 |
|------|------|
| `~/huaqiccc_simplified_flight_test.py` | Python 飞行测试脚本，发布轨迹 + 31440 变形命令 |
| `~/run_simplified_test.sh` | 一键启动仿真 + 运行测试 + 收集日志 |

### 4.3 被修改的 Board Config
| 文件 | 修改内容 |
|------|---------|
| `boards/px4/sitl/default.px4board` | 禁用非 MC 模块 |
| `boards/px4/fmu-v6c/default.px4board` | 禁用 UAVCAN, ROS2, 相机, 云台, 多余传感器驱动 |

---

## 5. 仿真调用方法

### 5.1 一键飞行测试
```bash
cd ~ && ./run_simplified_test.sh
```
流程：
1. `pkill` 清理旧进程
2. `roslaunch mavros_posix_sitl.launch` 启动 Gazebo + PX4 SITL
3. 等待日志中出现 `"Ready for takeoff!"`（约 20-30s）
4. 运行 `python3 huaqiccc_simplified_flight_test.py`
5. 轨迹：悬停 → 变形 → 圆×3 → 降落
6. 保存 CSV 日志到 `~/huaqiccc_logs/`

### 5.2 手动启动 SITL
```bash
cd ~/PX4-Autopilot
export PX4_SIM_MODEL=gazebo-classic_huaqiccc
make px4_sitl gazebo-classic_huaqiccc
```

### 5.3 编译验证
```bash
# SITL
make px4_sitl_default

# V6C 固件
make px4_fmu-v6c_default
```

---

## 6. 数据验证方法

### 6.1 飞行日志分析
```bash
python3 -c "
import pandas as pd
df = pd.read_csv('/home/a/huaqiccc_logs/huaqiccc_flight_with_algo_*.csv')
df['error_3d'] = (df['err_x']**2 + df['err_y']**2 + df['err_z']**2)**0.5
df['error_xy'] = (df['err_x']**2 + df['err_y']**2)**0.5
print(f'XY mean: {df[\"error_xy\"].mean():.4f} m')
print(f'3D mean: {df[\"error_3d\"].mean():.4f} m')
"
```

### 6.2 判断标准
| 指标 | 可接受范围 | 当前最佳 |
|------|-----------|---------|
| XY 平均误差 | < 0.15 m | **0.077 m** |
| 3D 平均误差 | < 0.25 m | **0.155 m** |
| 31440 命令成功率 | 100% | **19/19** |

---

## 7. 当前状态

### 7.1 FLASH 使用率（V6C）
- **原始**：98.64% (1,939,284 B)
- **当前**：**74.25%** (1,459,904 B)
- **节省**：约 480 KB

### 7.2 编译状态
- ✅ SITL 编译通过
- ✅ V6C 编译通过
- ✅ 飞行测试通过（变形指令正常，轨迹跟踪稳定）

### 7.3 保留的硬件支持
| 硬件 | 状态 |
|------|------|
| IMU (BMI055 + ICM42688P) | ✅ 保留 |
| Baro (MS5611) | ✅ 保留 |
| Mag (IST8310) | ✅ 保留 |
| GPS (M10, UART) | ✅ 保留 |
| 测距传感器 | ✅ 保留（可能用于定高） |
| PWM/DShot | ✅ 保留 |
| Logger | ✅ 保留（需求） |
| MAVLink | ✅ 保留 |

### 7.4 已删除的硬件支持
| 硬件 | 状态 |
|------|------|
| CAN / UAVCAN | ❌ 已删除 |
| ROS2 / uXRCE-DDS | ❌ 已删除 |
| 相机 / 云台 | ❌ 已删除 |
| 空速传感器 | ❌ 已删除 |
| 光流传感器 | ❌ 已删除 |
| FrSky/HoTT 遥测 | ❌ 已删除 |
| 多余磁力计 (11 种) | ❌ 已删除 |

---

## 8. 注意事项

1. **SITL vs 实机 KM 符号差异**：SITL 使用 `{-0.05, +0.05, +0.05, -0.05}`，实机 airframe 用 `+0.06`（被 LUT 覆盖）
2. **MPC 参数注入方式**：SITL 通过 `px4-rc.params` 注入；实机通过 airframe 文件或 QGroundStation
3. **MAVROS ParamSet 在 SITL 中失败**：`success=False`，所有调参通过启动脚本注入
4. **V6C 启动脚本**：`rc.board_sensors` 只启动 BMI055, ICM42688P, MS5611, IST8310
5. **如需恢复删除的文件**：`~/huaqiccc_backup_v2_20260519/restore.sh` 或 `~/PX4-Autopilot_pruned_backup_20260519_053000/`

---

## 9. 下一步可能的工作

- [ ] 实机试飞验证
- [ ] 变形策略优化（根据飞行状态自适应调整 arm_angle）
- [ ] 更激进的 MPC 调参（现在有 25%+ FLASH 余量，可以考虑启用更多功能）
- [ ] 添加更多传感器融合（如视觉定位）
- [ ] 距离传感器型号确认（根据实际挂载的硬件精确启用对应驱动）
