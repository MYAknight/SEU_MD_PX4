# huaqiccc 变形无人机测试套件

## 快速开始

```bash
# 主入口脚本（保留在 home 目录）
~/run_huaqiccc_test.sh <test_name> [args...]

# 示例
./run_huaqiccc_test.sh flatness 3 1          # MPC+FF 圆轨迹测试
./run_huaqiccc_test.sh simplified 0          # 原始 PID 简化飞行
./run_huaqiccc_test.sh batch_flatness        # 批量对比所有控制模式
```

---

## 目录结构

```
huaqiccc_test_suite/
├── README.md                          # 本文件
├── runners/                           # Shell 启动脚本
│   ├── 01_flatness_circle.sh          # 圆轨迹 + 平坦性前馈 (主测试)
│   ├── 02_simplified_flight.sh        # 简化飞行测试
│   ├── 03_aggressive_trajectory.sh    # 激进轨迹测试
│   ├── 11_pole_collision.sh           # 栖落碰撞测试
│   ├── 12_pole_pass_verify.sh         # 杆穿越验证测试
│   ├── 13_grasp_16cm.sh               # 16cm 杆抓取测试
│   ├── 21_batch_flatness_comparison.sh    # 批量对比: PID/GS-PID/LQR/MPC±FF
│   ├── 22_batch_aggressive_repeated.sh    # 批量激进轨迹 (2轮×5配置)
│   └── 23_mpc_parameter_sweep.sh          # MPC 参数扫参
├── flight/                            # 飞行测试 Python 脚本
│   ├── flatness_circle.py             # v4.1 圆轨迹 + 31440 + 速度/加速度前馈
│   ├── simplified_flight.py           # 简化版飞行测试
│   └── aggressive_trajectory.py       # 激进轨迹变体
└── perching/                          # 栖落测试 Python 脚本
    ├── pole_collision.py              # 杆碰撞 / 接触检测验证
    ├── pole_pass_verify.py            # 杆穿越几何验证
    └── grasp_16cm.py                  # 16cm 直径杆抓取

huaqiccc_tools/                        # 辅助工具
├── diagnose_and_launch.sh             # 仿真链路一键诊断与启动
├── fault_detector.py                  # 实时失控监控与诊断
├── flatness_verify.py                 # 平坦性前馈代数自洽验证
├── flight_analyzer.py                 # 飞行数据后处理分析
├── batch_analyzer.py                  # 批量实验数据分析
├── perching_log_analyzer.py           # 栖落测试 CSV 快速分析
└── evaluate_flight.py                 # 基础飞行评估指标

huaqiccc_tools/gui/                    # 机臂变形控制 GUI（被 diagnose_and_launch.sh 引用）
├── huaqiccc_deform_control_gui.py     # 机臂变形控制 GUI
├── huaqiccc_dynamic_param_adjust.py   # 动态参数调整
└── huaqiccc_unified_control_gui.py    # 统一控制 GUI
```

---

## 测试说明

### 飞行测试 (flight/)

| 脚本 | 功能 | 典型参数 |
|------|------|----------|
| `01_flatness_circle.sh` | 圆轨迹飞行 + 31440 变形命令 + 速度/加速度前馈 | `MPCA_MODE` `MPCA_FF_EN` |
| `02_simplified_flight.sh` | 简化版飞行，支持 4 种模式切换 | `MPCA_MODE` |
| `03_aggressive_trajectory.sh` | 大半径快速圆轨迹，测试跟踪极限 | `MPCA_MODE` `MPCA_FF_EN` |

### 栖落测试 (perching/)

| 脚本 | 功能 | 备注 |
|------|------|------|
| `11_pole_collision.sh` | 缓慢推向杆体，验证 IMU-ICD 接触检测 | 使用 IMU 冲击检测 FSM |
| `12_pole_pass_verify.sh` | 穿越杆体几何验证 | 测试变形后几何通过性 |
| `13_grasp_16cm.sh` | 16cm 杆抓取与栖落 | 张开臂 → 慢推 → 检测 → 锁定 |

### 批量测试 (batch/)

| 脚本 | 功能 | 输出 |
|------|------|------|
| `21_batch_flatness_comparison.sh` | 对比 4 种控制模式 × ±前馈 | `~/huaqiccc_logs/huaqiccc_flatness_m*_ff*.csv` |
| `22_batch_aggressive_repeated.sh` | 5 种配置各跑 2 轮 | `~/huaqiccc_logs/huaqiccc_aggressive_*.csv` |
| `23_mpc_parameter_sweep.sh` | α (10~30) × r_Δ (0.002~0.01) 矩阵 | `~/huaqiccc_logs/huaqiccc_flatness_*.csv` |

---

## 控制模式对照表

| MPCA_MODE | 名称 | 说明 |
|-----------|------|------|
| 0 | Original PID | PX4 原始位置控制器 |
| 1 | GS-PID | 增益调度 PID |
| 2 | LQR | 线性二次调节器 |
| 3 | MPC+FF | 模型预测控制 + 平坦性前馈 (推荐) |

---

## 环境要求

- ROS Noetic
- Gazebo Classic 11
- MAVROS 1.20+
- PX4 SITL (已编译 `px4_sitl_default`)
- `huaqiccc_ws` Gazebo 插件

---

## 旧版脚本归档

不再维护的历史脚本已移至：

```
~/backups/dev_snapshots/old_test_scripts/
├── downloads/              # Downloads 目录旧版脚本
├── morphing_patch/         # 早期变形补丁 (已集成到固件)
└── old_plugin/             # 旧版 ROS Gazebo 插件
```

---

## 更新日志

- **2026-05-27**: 统一整理测试脚本，建立 `huaqiccc_test_suite/` 目录结构
- **2026-05-27**: 设置 `posix_sitl.launch` `interactive="false"` 消除 `pxh>` 刷屏
