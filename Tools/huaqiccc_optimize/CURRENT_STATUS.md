# Huaqiccc 变形无人机 — 当前进度总结

> 生成时间：2026-06-14  
> 本文件记录本次对话完成的所有工作，用于后续对话快速恢复上下文。
>
> **USER_CONFIG / 路径迁移提示**
>
> 本工具链原位于 `~/Projects/optimize`，现已并入 `SEU_MD_PX4/Tools/huaqiccc_optimize/`。
> `download_logs.py`、`analyze_flight.py`、`safe_space_2x2x2.yaml` 中仍保留原开发环境
> 的默认日志路径。首次部署到新机器时，请搜索 `USER_CONFIG` 并按实际路径修改。
> 详见 `Tools/REALFLIGHT_PATH_MIGRATION.md`。

---

## 一、本次对话完成的工作

### 1. 修复了代码中的已知问题

| 文件 | 问题 | 修复内容 |
|------|------|---------|
| `YOLO4SEU_MD/rtsp_pillar_node.py` | 节点初始化日志和信号处理注册被错误放在回调函数中 | 移到 `__init__()` 末尾 |
| `PX4/SEU_MD_PX4/src/modules/huaqiccc_morph_control/huaqiccc_morph_control_params.c` | `MORPH_KP`、`MORPH_DB` 参数定义但未使用 | 删除这两个参数 |
| `PX4/SEU_MD_PX4/src/modules/huaqiccc_morph_control/HuaqicccMorphControl.hpp` | `MORPH_KP`、`MORPH_DB` 参数声明 | 删除声明 |
| `PX4/SEU_MD_PX4/src/modules/control_allocator/huaqiccc_motor_lut.hpp` | motor 顺序注释笔误（`3=rf` 应为 `3=lf`） | 修正注释 |
| `PX4/SEU_MD_PX4/src/modules/control_allocator/huaqiccc_motor_lut.hpp` | arm_angle 范围描述 | 统一为 0.0 (closed) ~ -0.40 rad |
| `PX4/SEU_MD_PX4/src/modules/huaqiccc_morph_control/huaqiccc_morph_control_params.c` | `MORPH_EMAX` 注释写 `-0.5 rad` | 修正为 `-0.4 rad` |
| `YOLO4SEU_MD/config/detector.yaml` | 相机高度 480 | 统一为 360 |
| `YOLO4SEU_MD/pillar_detector/rtsp_detector.py` | 默认 camera_height 480 | 统一为 360 |

### 2. 更新了项目文档

| 文件 | 更新内容 |
|------|---------|
| `~/Projects/README.md` | 当前阶段改为"实机接触检测验证完成"，新增 2026-06-14 遥测通道与验证结果 |
| `~/Projects/ground_station/AGENTS.md` | 新增接触/栖停遥测数据流与数组定义，更新固件/待办状态 |
| `~/Projects/ground_station/README.md` | 新增接触/栖停状态面板说明与 `/mavros/debug_value/debug_float_array` topic |
| `~/Projects/PX4/SEU_MD_PX4/CURRENT_STATUS.md` | 新增 `DEBUG_FLOAT_ARRAY` 遥测通道章节，标记接触检测实机验证完成 |
| `~/Projects/PX4/SEU_MD_PX4/PROJECT_STATUS_REPORT.md` | 实机接触检测状态改为 ✅ |
| `~/Projects/PX4/SEU_MD_PX4/BUG_FIX_RECORD.md` | Bug #9 实机飞行验证改为 ✅，新增遥测通道说明 |
| `~/Projects/PX4/SEU_MD_PX4/HARDWARE_MIGRATION_GUIDE.md` | 刷新为 V6C 实机目标，更新遥测数据流与下一步建议 |
| `~/Projects/YOLO4SEU_MD/README_RTSP_MIGRATION.md` | 流程图中 Stall Detection 改为位置/姿态接触检测 |
| `~/Projects/optimize/plan.md` | 更新任务 2：移除 GMO 标定，修正 `MPCA_PC_EN` 使用 |

### 3. 遥测通道重构与实机接触检测验证（2026-06-14）

| 文件 | 修改内容 |
|------|---------|
| `PX4/SEU_MD_PX4/src/modules/mc_pos_control/MulticopterPositionControl.cpp/.hpp` | 发布 `DEBUG_FLOAT_ARRAY`（`name="perch"`），包含接触状态、栖停阶段、变形臂角度等 |
| `ground_station/scripts/control_ground_station_ros.py` | 新增 `/mavros/debug_value/debug_float_array` 订阅与解析，保留 statustext 备用 |

**实机验证结果**：
- ✅ V6C 固件编译通过（FLASH 77.87%）并刷入 Pixhawk 6C
- ✅ 手动栖停测试：位置/姿态接触检测灵敏度合适
- ✅ 地面站可实时查看接触/栖停状态，遥测链路稳定

### 4. 创建了 `~/Projects/optimize/` 自动化测试工具链

| 文件 | 作用 |
|------|------|
| `launch_env.sh` | 一键启动完整环境：roscore + VRPN + MAVROS + throttle + flight_executor |
| `start_optimize_station.sh` | 一键启动环境 + GUI（同 start_ground_station.sh 风格） |
| `auto_flight_gui.py` | 简化飞行 GUI：轨迹选择 + 一键起飞/降落/急停 + 日志下载 |
| `flight_executor.py` | 飞行执行器 ROS 节点，提供 `/optimize/start_flight`, `/optimize/land`, `/optimize/emergency_stop` 服务 |
| `safe_space.yaml` | 安全空间配置：实际测量四边形角点 + home 点 + 高度/速度/裕量参数 |
| `verify_safe_space.py` | 地面前验证：绘制安全空间与轨迹，检查最小边界距离 |
| `download_logs.py` | 通过 MAVLink FTP 下载最新 ulog，无需 QGC |
| `analyze_flight.py` | 自动分析飞行日志，生成自然语言报告和优化建议 |
| `requirements.txt` | 依赖说明 |

---

## 二、当前项目状态

### 已验证（2026-06-11 实机飞行）

- ✅ 6月11日最新固件已刷入 Pixhawk 6C
- ✅ 正常起飞
- ✅ 位置模式（POSCTL）稳定飞行
- ✅ OFFBOARD 控制响应正常
- ✅ 变形功能在飞行中可靠工作
- ✅ LUT 查表在变形过程中实时更新正常
- ✅ 四种控制算法实机验证：PID / GS-PID / LQR / MPC
- ✅ AS5600 编码器反馈正常
- ✅ 地面站通过 ROS/MAVROS 与飞控通信正常

### 正在进行

- 自主接触抱住测试准备（`MPCA_PC_EN=2`，人工监控）
- 控制参数调优（建立实机稳定参数基线）

### 待开始

- YOLO 视觉 YAW 对齐与地面站集成
- 一键栖停任务自动化

---

## 三、实际测量安全空间测试准备

### 安全空间配置（`~/Projects/optimize/safe_space.yaml`）

```yaml
safe_space:
  corners:
    - [1.233, 1.542, 0.1]    # 左前
    - [-0.433, 1.337, 0.1]   # 左后
    - [-0.5, -0.246, 0.1]    # 右后
    - [1.432, -0.305, 0.1]   # 右前
  home: [0.332, 0.727, 0.1]  # 起飞/降落中心
  z_min: 0.1
  z_max: 2.0

mission:
  flight_height: 1.0
  safety_margin: 0.30        # 轨迹/飞行器距边界 ≥ 0.30 m
```

home 点到四边形边界最短距离约 **0.699 m**。扣除 0.30 m 安全裕量后，轨迹可用最大半径约 **0.40 m**。当前轨迹尺寸统一控制在半径/半长 **≤ 0.30 m**。

### 可用轨迹

| 轨迹 | 尺寸 | 速度 | 用途 |
|------|------|------|------|
| `hover` | 中心点 | 0 | 稳态悬停 |
| `takeoff_land` | 中心垂直 | 0.2 m/s | 起降测试 |
| `square_small` | 边长约 0.40 m | 0.3 m/s | XY 轴向位置跟踪 |
| `circle_small` | 半径 0.30 m | 0.3 m/s | 连续曲线跟踪 |
| `figure8_small` | 半径 0.30 m 的 8 字 | 0.3 m/s | 连续变向耦合 |
| `step_x` | ±0.22 m | 0.3 m/s | X 轴阶跃响应 |
| `step_xy` | 对角 ±0.22 m | 0.3 m/s | XY 耦合响应 |
| `morph_circle` | 半径 0.30 m 的圆 | 0.3 m/s | 变形中位置跟踪 |

**地面前验证结果**（`python3 verify_safe_space.py`）：
- 所有轨迹最小边界距离 ≥ **0.399 m**，满足 0.30 m 安全裕量。
- 可视化图已保存：`~/Projects/optimize/safe_space_verify.png`

### 使用流程

```bash
cd ~/Projects/optimize

# 方式 1：分别启动环境和 GUI
./launch_env.sh
# 另一个终端
python3 auto_flight_gui.py

# 方式 2（推荐）：一键启动环境 + GUI
./start_optimize_station.sh

# GUI 中选择轨迹和 MPCA_MODE，点击"一键起飞并执行轨迹"

# 4. 飞行结束后下载日志
python3 download_logs.py

# 5. 分析日志
python3 analyze_flight.py
```

---

## 四、降落模式失败问题的解决方案

之前旧版启动脚本中使用降落模式失败，可能原因是 OFFBOARD setpoint 与 LAND 模式竞争。该脚本已归档到 `~/Projects/backup/ground_station_deprecated_2026-06-14/`。

`flight_executor.py` 中的降落流程：
1. 通过 setpoint 缓慢下降到离地 0.3m
2. 稳定 2 秒
3. **停止发布 OFFBOARD setpoint 后**，切换到 `LAND` 模式
4. 等待高度接近地面或自动 DISARM
5. 如果 LAND 失败，自动 fallback 到 `POSCTL`

---

## 五、关键文件路径索引

### 项目总览
- `~/Projects/README.md` — 父目录导航
- `~/Projects/optimize/CURRENT_STATUS.md` — 本文件
- `~/Projects/optimize/plan.md` — 下阶段完整计划

### PX4 固件
- `~/Projects/PX4/SEU_MD_PX4/` — 定制 PX4 v1.14.3
- `~/Projects/PX4/SEU_MD_PX4/CURRENT_STATUS.md` — 固件状态
- `~/Projects/PX4/SEU_MD_PX4/src/modules/huaqiccc_morph_control/` — 变形控制
- `~/Projects/PX4/SEU_MD_PX4/src/modules/control_allocator/huaqiccc_motor_lut.hpp` — LUT

### 视觉
- `~/Projects/YOLO4SEU_MD/` — YOLO 视觉模块
- `~/Projects/YOLO4SEU_MD/rtsp_pillar_node.py` — YAW 对齐节点

### 地面站
- `~/Projects/ground_station/` — 地面站
- `~/Projects/ground_station/scripts/control_ground_station_ros.py` — 当前在用控制地面站

### 自动化测试工具
- `~/Projects/optimize/launch_env.sh`
- `~/Projects/optimize/auto_flight_gui.py`
- `~/Projects/optimize/flight_executor.py`
- `~/Projects/optimize/safe_space.yaml`
- `~/Projects/optimize/verify_safe_space.py`
- `~/Projects/optimize/download_logs.py`
- `~/Projects/optimize/analyze_flight.py`

---

## 六、下一步行动建议

1. **自主接触抱住测试**：在监控下启用 `MPCA_PC_EN=2`，验证程序自动进入 Perching FSM
2. **验证 `launch_env.sh`**：确认环境能正常启动
3. **测试 `hover` / `circle_small` 轨迹**：确认 offboard 轨迹执行稳定
4. **提供当前稳定参数文件**：用于后续自动化参数调优
5. **执行参数调优**：参考 `plan.md` 任务 1

---

*本文件应随每次对话更新，确保后续对话能快速恢复上下文。*
