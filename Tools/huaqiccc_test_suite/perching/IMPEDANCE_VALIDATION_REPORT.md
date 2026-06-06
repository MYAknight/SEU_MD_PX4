# 阻抗控制效果验证报告

> 生成日期：2026-05-28
> 评价对象：MPCA_PC_K_SOFT（位置环P增益在COMPLIANT阶段的乘数）

---

## 1. 评价指标体系

设计了5个量化指标，覆盖接触瞬间、接触后稳定、最终精度三个时间维度：

| 指标 | 符号 | 定义 | 数据来源 | 理想趋势 |
|------|------|------|---------|---------|
| **接触冲击** | CI | stall_detect前后±0.3s内IMU加速度峰值 `max(a_norm)` | `/mavros/imu/data` | **↓ 越小越柔顺** |
| **接触后振荡(X)** | PCO_X | contact后[t+2, t+4]s内X位置标准差 | `/mavros/local_position/pose` | **↓ 越小越稳定** |
| **接触后振荡(Z)** | PCO_Z | contact后[t+2, t+4]s内Z位置标准差 | `/mavros/local_position/pose` | **↓ 越小越稳定** |
| **Monitor精度(Z)** | MON_Z | monitor阶段Z位置标准差 | `/gazebo/model_states` | **↓ 越小越精确** |
| **成功率** | SR | monitor判定为SUCCESS的占比 | 综合 | **↑ 越高越好** |

---

## 2. 闭环测试流程

### 2.1 自动化脚本

创建了3个配套脚本：

| 脚本 | 功能 |
|------|------|
| `impedance_benchmark.py` | 自动切换`MPCA_PC_K_SOFT`，循环运行多组实验，每次重启SITL确保公平 |
| `analyze_impedance.py` | 批量读取CSV，自动计算5项指标，输出对比报告 |
| `grasp_16cm.py` (已更新) | 新增IMU加速度记录（`ax, ay, az, a_norm`字段） |

### 2.2 实验设计

```
Group A (对照):  k_soft = 1.00  (阻抗关闭，等效原始代码)
Group B (实验):  k_soft = 0.20  (默认阻抗，P增益降至20%)
Group C (边界):  k_soft = 0.05  (极柔，P增益降至5%)

每组运行 N_TRIALS = 3 次
每次测试前重启SITL，消除EKF漂移影响
```

### 2.3 一键执行

```bash
cd ~/huaqiccc_test_suite/perching

# 运行完整benchmark（约15-20分钟）
python3 impedance_benchmark.py

# 分析结果
python3 analyze_impedance.py --prefix impedance
```

---

## 3. 初步实验结果

由于单次完整benchmark耗时较长，先执行了快速对比实验（每组1次）：

### 3.1 原始数据

| 组别 | k_soft | SITL状态 | CI (m/s²) | PCO_X (m) | PCO_Z (m) | MON_Z_std (m) | 结果 |
|------|--------|---------|-----------|-----------|-----------|---------------|------|
| A | 1.00 | 未重启（旧会话） | 14.96 | 2.60 | 0.51 | 0.669 | ⚠️ EKF漂移，数据不可靠 |
| B | 0.20 | 重启后（新会话） | 13.44 | 2.60 | 0.27 | 0.146 | ✅ 正常 |

### 3.2 结果解读

**关键发现：**

1. **CI（接触冲击）**：k_soft=0.20 的 CI = 13.44 m/s²，比 k_soft=1.00 的 14.96 m/s² **低10%**
   - 说明阻抗控制确实降低了接触瞬间的加速度峰值

2. **PCO_Z（Z向振荡）**：k_soft=0.20 的 PCO_Z = 0.27 m，比 k_soft=1.00 的 0.51 m **低48%**
   - 说明阻抗控制显著改善了接触后的高度稳定性

3. **MON_Z_std**：k_soft=0.20 的 monitor Z 标准差 = 0.146 m，远小于 k_soft=1.00 的 0.669 m
   - 说明有阻抗控制时，最终栖停位置更稳定

**但需要注意：**
- Group A（k_soft=1.00）的数据来自**未重启的SITL会话**，EKF已漂移，导致monitor数据不可靠
- 两组测试的SITL状态不一致，**不能直接作为统计结论**
- 需要每组3次、每次重启SITL的完整实验，才能得出可靠结论

---

## 4. 已知局限

### 4.1 实验侧

1. **样本量不足**：仅1次对比，缺乏统计显著性
2. **SITL会话干扰**：连续测试不重启SITL会导致EKF漂移，污染数据
3. **IMU数据频率**：MAVROS IMU约50Hz，但Python脚本仅20Hz记录，可能遗漏峰值

### 4.2 指标侧

1. **CI受噪声影响**：IMU加速度包含电机振动噪声，峰值可能不完全反映接触力
2. **PCO窗口选择**：[t+2, t+4]是经验值，可能需要根据实际收敛速度调整
3. **缺少真实接触力**：Gazebo无法直接输出接触力，只能用加速度近似

### 4.3 系统侧

1. **Gazebo freeze掩盖差异**：plugin auto-fix在1.5s内冻结模型，缩短了阻抗控制的作用时间窗口
2. **仿真物理限制**：Gazebo的碰撞模型与实机差异大，仿真结果只能定性参考

---

## 5. 代码实现确认

### 5.1 实现位置

```cpp
// MulticopterPositionControl.cpp ~164行
Vector3f pos_gain(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get());
if (_perching_phase == PerchingPhase::COMPLIANT) {
    pos_gain *= _param_mpca_pc_k_soft.get();
}
_control.setPositionGains(pos_gain);

// _advanced_control 同样处理
```

### 5.2 生效确认

PX4 日志输出：
```
INFO  [mc_pos_control] Perching: entering soft contact, impedance k_soft=0.20
```

确认 FSM 进入 COMPLIANT 阶段时，k_soft=0.20 被正确应用。

---

## 6. 后续建议

### 6.1 立即执行（30分钟）

运行完整benchmark：
```bash
cd ~/huaqiccc_test_suite/perching
python3 impedance_benchmark.py
```

### 6.2 结果分析（10分钟）

```bash
python3 analyze_impedance.py --prefix impedance
```

### 6.3 判定标准

如果 Group B（k_soft=0.20）相比 Group A（k_soft=1.00）满足：
- CI 降低 ≥ 10% **且**
- PCO_Z 降低 ≥ 30% **且**
- SR ≥ 66%

则判定**阻抗控制具有统计显著的积极作用**。

### 6.4 参数调优（如需）

如果 k_soft=0.20 效果不佳，可尝试：
- 0.30：较柔和，保持更多刚度
- 0.10：更柔顺，但可能响应过慢

---

*报告版本：v1.0*
*状态：评价标准和测试流程已完成，待执行完整benchmark*
