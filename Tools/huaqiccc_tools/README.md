# huaqiccc 辅助工具集

## 工具一览

| 工具 | 功能 | 用法 |
|------|------|------|
| `diagnose_and_launch.sh` | 一键诊断仿真环境并启动 | `./diagnose_and_launch.sh [diag\|launch\|gui]` |
| `fault_detector.py` | 实时失控监控（姿态震荡、电机饱和、EKF 偏差） | `rosrun 或 python3 fault_detector.py` |
| `flatness_verify.py` | 平坦性前馈代数自洽验证（离线数学验证） | `python3 flatness_verify.py` |
| `flight_analyzer.py` | 单轮飞行 CSV 后处理分析 | `python3 flight_analyzer.py <csv_file>` |
| `batch_analyzer.py` | 批量实验数据对比分析 | `python3 batch_analyzer.py <log_dir>` |
| `perching_log_analyzer.py` | 栖落测试 CSV 快速分析 | `python3 perching_log_analyzer.py <csv_file>` |
| `evaluate_flight.py` | 基础飞行评估（被批量脚本调用） | `python3 evaluate_flight.py <csv_file>` |

## 诊断脚本说明

### diagnose_and_launch.sh

```bash
./diagnose_and_launch.sh diag    # 仅诊断，不启动
./diagnose_and_launch.sh launch  # 诊断 + 启动 SITL
./diagnose_and_launch.sh gui     # 启动变形控制 GUI
```

检查项：
- ROS 环境
- PX4 编译状态
- Gazebo 插件
- MAVROS 连接
- huaqiccc 模型文件

### fault_detector.py

实时检测以下异常：
1. 位置跟踪误差 > 阈值
2. 姿态角速度异常
3. 电机指令饱和
4. EKF 与 GroundTruth 偏差（SITL 专用）
5. 变形关节角度不同步

---

*整理日期: 2026-05-27*
