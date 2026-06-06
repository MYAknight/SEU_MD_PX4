# 新对话回顾性测试指南

> 本文件用于新对话开始时快速回顾项目状态。
> 按照以下步骤执行，可在5分钟内确认系统当前状态。

---

## 快速状态检查（5分钟）

### Step 1：确认文件存在（30秒）

```bash
# 检查关键文件是否存在
ls ~/PX4-Autopilot/src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.cpp
ls ~/PX4-Autopilot/src/modules/mc_pos_control/FlatnessFeedforward/FlatnessFeedforward.hpp
ls ~/PX4-Autopilot/src/modules/external_force_estimator/external_force_estimator.cpp
ls ~/PX4-Autopilot/src/modules/control_allocator/huaqiccc_motor_lut.hpp
ls ~/run_huaqiccc_test.sh
ls ~/huaqiccc_test_suite/flight/flatness_circle.py
ls ~/PX4-Autopilot/大纲0418/章节草稿/第2-3章_实机版_逻辑通顺版.md
ls ~/PX4-Autopilot/回顾总结_20260527/PROJECT_PROGRESS_SUMMARY.md
```

**预期结果**：所有文件存在。

### Step 2：确认代码可编译（2分钟）

```bash
cd ~/PX4-Autopilot
make px4_sitl_default -j$(nproc) 2>&1 | tail -5
```

**预期结果**：`[100%] Built target px4` 或类似成功信息，无错误。

### Step 3：确认参数配置（30秒）

```bash
cat ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params | grep -E "MPCA|MPC_ALPHA|MPC_R_DELTA"
```

**预期结果**：
```
param set-default MPCA_MODE 2
param set-default MPCA_MPC_ALPHA 20.0
param set-default MPCA_MPC_R_DELTA 0.005
param set-default MPCA_FF_EN 1
param set-default MPCA_FF_BLEND 0.3
param set-default MPCA_FF_MASS 1.5
```

### Step 4：确认历史实验数据存在（30秒）

```bash
ls -lt ~/huaqiccc_logs/huaqiccc_aggressive_*_r*.csv 2>/dev/null | head -10
```

**预期结果**：显示10个文件（mpc_ff_r1/r2, mpc_noff_r1/r2, lqr_r1/r2, basic_r1/r2, orig_r1/r2）。

### Step 5：快速复现验证（2分钟）

```bash
# 运行一个快速测试（可选，会启动SITL）
cd ~ && export MPCA_MODE=3 && export MPCA_FF_EN=1 && export MPCA_FF_BLEND=0.3 &&
export MPCA_MPC_ALPHA=20.0 && export MPCA_MPC_R_DELTA=0.005 &&
bash run_huaqiccc_test.sh flatness 3 1
```

**预期结果**：
- Gazebo启动，无人机在地面
- 测试脚本运行约52秒
- 生成新的CSV文件到~/huaqiccc_logs/
- 无失控、无报错

---

## 深度状态检查（如需验证特定功能）

### 验证A：Flatness Feedforward是否生效

```bash
# 方法1：检查MPC+FF模式下的CSV数据
cd ~/huaqiccc_logs && python3 -c "
import pandas as pd, numpy as np
# 读取最新MPC+FF文件
import glob
f = sorted(glob.glob('huaqiccc_flatness_m3_ff1_*_with_algo_*.csv'))[-1]
df = pd.read_csv(f)
t = df['time'].values
mask = (t - t[0] >= 18) & (t - t[0] <= 38)
err = np.sqrt(df['err_x'][mask]**2 + df['err_y'][mask]**2)
print(f'MPC+FF circle_xy RMSE: {np.sqrt(np.mean(err**2)):.4f} m')
"
```

**预期结果**：circle_xy RMSE ≈ 0.08-0.12m（正常范围）。

### 验证B：控制效率矩阵LUT是否正确更新

```bash
# 方法：检查PX4编译输出中是否包含LUT文件
grep -n "huaqiccc_motor_lut" ~/PX4-Autopilot/src/modules/control_allocator/ControlAllocator.cpp | head -3
```

**预期结果**：显示`huaqiccc_get_motor_params`调用行。

### 验证C：IMU-ICD FSM是否正确运行

```bash
# 方法：检查uORB话题是否存在
ls ~/PX4-Autopilot/msg/ContactState.msg
ls ~/PX4-Autopilot/msg/ExternalForceEstimate.msg
ls ~/PX4-Autopilot/msg/HuaqicccMorphAngle.msg
```

**预期结果**：三个msg文件均存在。

### 验证D：MAVROS加速度修复是否生效

```bash
# 方法：检查mavlink_receiver.cpp中的修复代码
grep -n "setpoint.acceleration\[0\]" ~/PX4-Autopilot/src/modules/mavlink/mavlink_receiver.cpp
```

**预期结果**：显示包含`target_local_ned.afx`的代码行。

---

## 项目当前阶段判断

根据以下检查项，判断项目所处阶段：

| 检查项 | 阶段1：代码开发 | 阶段2：仿真验证 | 阶段3：实机实验 | 阶段4：论文定稿 |
|--------|---------------|---------------|---------------|---------------|
| 代码编译通过 | ✅ | ✅ | ✅ | ✅ |
| SITL测试数据完整 | 部分 | ✅ | ✅ | ✅ |
| 实机飞行数据 | ❌ | ❌ | ⏳ | ✅ |
| 栖息成功率数据 | ❌ | ❌ | ⏳ | ✅ |
| 振动检测数据 | ❌ | ❌ | ⏳ | ✅ |
| 论文章节完整 | ❌ | 初稿 | 初稿 | ✅ |

**当前判断（2026-05-27）**：阶段2（仿真验证完成）→ 阶段3（实机实验待开始）

---

## 常见问题的快速诊断

### 问题1：编译失败

```bash
# 检查是否缺少子模块
cd ~/PX4-Autopilot && git submodule update --init --recursive
# 清理构建
cd ~/PX4-Autopilot && rm -rf build/px4_sitl_default && make px4_sitl_default
```

### 问题2：MAVROS连接失败

```bash
# 检查launch文件中的fcu_url
grep "fcu_url" ~/PX4-Autopilot/launch/mavros_posix_sitl.launch
# 应为：udp://:14540@localhost:14580
```

### 问题3：参数未生效（总是运行上一次的配置）

```bash
# 删除保存的参数文件
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters_backup.bson
```

### 问题4：找不到实验数据

```bash
# 检查日志目录
ls -lt ~/huaqiccc_logs/*.csv | head -5
# 检查ULOG
ls -lt ~/.ros/log/*/*.ulg | head -5
```

---

## 与新AI助手对话时的最佳实践

### 首次对话时，请提供以下信息

1. **当前阶段**：仿真验证完成 / 实机实验进行中 / 论文撰写中
2. **本次目标**：例如"需要补充实机栖息数据"、"修改论文第二章"、"调试MPC参数"
3. **已知限制**：例如"实机尚未安装接触点加速度传感器"、"外场实验 pending"

### 让AI快速进入状态的方法

```
请阅读以下文件以了解项目背景：
1. ~/PX4-Autopilot/回顾总结_20260527/PROJECT_PROGRESS_SUMMARY.md （项目总览）
2. ~/PX4-Autopilot/回顾总结_20260527/BUG_FIX_RECORD.md （关键Bug）
3. ~/PX4-Autopilot/大纲0418/大纲0418.md （论文大纲）

当前目标：[填写具体目标]
```

### 避免的信息缺失

- ❌ "帮我修改代码" → 不说明修改哪里
- ❌ "运行测试" → 不说明测试什么
- ❌ "分析一下" → 不说明分析对象

- ✅ "帮我修改论文第2.2节，将LUT更新描述从最近邻改为线性插值"
- ✅ "运行MPC参数调优测试，alpha取10/15/20/25"
- ✅ "分析LQR在实机上的hover误差偏高的原因"

---

*文档版本：v1.1*
> 更新：2026-05-27 同步测试套件重构与参数默认值
*生成时间：2026-05-27*
*适用PX4版本：v1.14.3 + huaqiccc modifications*
EOF
echo "新对话指南已生成"