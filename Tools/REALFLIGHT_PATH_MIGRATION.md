# 实机地面站/调优工具路径迁移指南

> 本仓库原本在 `/home/a/Projects/PX4/SEU_MD_PX4` 下开发，地面站原位于 `/home/a/Projects/ground_station`，调优工具原位于 `/home/a/Projects/optimize`。为了便于迁移到其他设备，已将地面站和调优工具移入本仓库：
>
> - `Tools/huaqiccc_ground_station/`
> - `Tools/huaqiccc_optimize/`
>
> 以下列出所有仍然写死原开发环境路径的地方。首次部署到新机器时，请按本指南搜索并修改。

---

## 一、地面站 `Tools/huaqiccc_ground_station/`

### 1.1 启动脚本

| 文件 | 搜索关键字 | 需要修改的内容 |
|------|-----------|---------------|
| `start_ground_station.sh` | `USER_CONFIG` | 最后一行 `python3 ~/Projects/...` 指向 control_ground_station_ros.py 的绝对路径 |
| `start_ground_station_auto.sh` | `USER_CONFIG` | 最后一行 `python3 ~/Projects/...` 指向 control_ground_station_auto.py 的绝对路径；`ROS_PACKAGE_PATH` 中 YOLO4SEU_MD 的绝对路径 |

修改建议：把 `~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_ground_station` 替换为你本机实际克隆路径；把 `~/Projects/YOLO4SEU_MD` 替换为你本机 YOLO4SEU_MD 路径。

### 1.2 Python 脚本

`scripts/control_ground_station_auto.py` 和 `scripts/control_ground_station_ros.py` 内部主要使用 ROS topic/service，**没有硬编码项目路径**。但要注意：

- `camera_offset_x` 默认从 `~/.config/ground_station/camera_offset.yaml` 加载（这是用户主目录下的固定位置，通常无需修改）。

### 1.3 文档

`README.md` 和 `AGENTS.md` 中多处出现 `~/Projects/ground_station`、`~/Projects/YOLO4SEU_MD`、`~/Projects/PX4/SEU_MD_PX4` 等示例路径。这些是说明文字，不影响运行，但阅读时请注意按你的实际路径理解。

---

## 二、调优工具 `Tools/huaqiccc_optimize/`

### 2.1 关键可执行文件

| 文件 | 搜索关键字 | 需要修改的内容 |
|------|-----------|---------------|
| `download_logs.py` | `USER_CONFIG` | `--output-dir` 默认路径：`~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_optimize/logs` |
| `analyze_flight.py` | `USER_CONFIG` | 默认 `log_path`：`~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_optimize/logs/latest.ulg` |
| `start_optimize_station.sh` | `USER_CONFIG` | 用法注释中的 `cd ~/Projects/optimize` |
| `safe_space_2x2x2.yaml` | `USER_CONFIG` | `logging.local_log_dir` 字段 |

### 2.2 说明文档

`CURRENT_STATUS.md` 和 `plan.md` 中保留大量 `~/Projects/optimize`、`~/Projects/ground_station` 等历史路径说明。它们只作为背景参考，不影响程序运行。

---

## 三、PX4 固件内部测试脚本

`Tools/huaqiccc_test_suite/` 下的部分 runner 和 Python 脚本也包含原开发机路径，例如：

- `runners/*.sh` 中的 `source /home/a/Projects/PX4/env_seu_md_px4.sh`
- `runners/*.sh` 中的 `source /home/a/catkin_ws/devel_isolated/setup.bash`
- Python 脚本中的 `GAZEBO_PLUGIN_PATH`、`GAZEBO_MODEL_PATH` 指向 `/home/a/Projects/PX4/SEU_MD_PX4/build/...`

这些脚本主要用于 SITL/Gazebo 仿真。若在新机器上运行，请按以下顺序修改：

1. 把 `/home/a/Projects/PX4/SEU_MD_PX4` 替换为你本机 SEU_MD_PX4 路径。
2. 把 `/home/a/catkin_ws` 替换为你本机 catkin 工作空间路径。
3. 把 `/home/a/Projects/PX4/env_seu_md_px4.sh` 替换为你本机 PX4 环境脚本路径（或删除/替换为等价的环境设置）。

---

## 四、快速替换命令示例

如果你把整个仓库克隆到 `~/repos/SEU_MD_PX4`，可批量替换（请先在副本上测试）：

```bash
REPO_ROOT=~/repos/SEU_MD_PX4
YOLO_ROOT=~/repos/YOLO4SEU_MD

# 地面站脚本
sed -i "s|~/Projects/PX4/SEU_MD_PX4|$REPO_ROOT|g" \
  $REPO_ROOT/Tools/huaqiccc_ground_station/start_ground_station*.sh
sed -i "s|~/Projects/YOLO4SEU_MD|$YOLO_ROOT|g" \
  $REPO_ROOT/Tools/huaqiccc_ground_station/start_ground_station_auto.sh

# 调优工具默认路径
sed -i "s|~/Projects/PX4/SEU_MD_PX4|$REPO_ROOT|g" \
  $REPO_ROOT/Tools/huaqiccc_optimize/download_logs.py \
  $REPO_ROOT/Tools/huaqiccc_optimize/analyze_flight.py \
  $REPO_ROOT/Tools/huaqiccc_optimize/safe_space_2x2x2.yaml

# SITL runner（可选，仅仿真时需要）
find $REPO_ROOT/Tools/huaqiccc_test_suite -type f \( -name "*.sh" -o -name "*.py" \) \
  -exec sed -i "s|/home/a/Projects/PX4/SEU_MD_PX4|$REPO_ROOT|g" {} +
```

---

## 五、首次部署检查清单

- [ ] 修改 `Tools/huaqiccc_ground_station/start_ground_station*.sh` 中的绝对路径
- [ ] 修改 `Tools/huaqiccc_ground_station/start_ground_station_auto.sh` 中的 `ROS_PACKAGE_PATH`
- [ ] 修改 `Tools/huaqiccc_optimize/download_logs.py` 默认输出目录
- [ ] 修改 `Tools/huaqiccc_optimize/analyze_flight.py` 默认日志路径
- [ ] 修改 `Tools/huaqiccc_optimize/safe_space_2x2x2.yaml` 中的 `local_log_dir`
- [ ] （可选）若运行 SITL，修改 `Tools/huaqiccc_test_suite/runners/*.sh` 中的环境路径
- [ ] 确认 `/dev/ttyUSB0`、VRPN 服务器 IP `192.168.1.5`、Tracker 名称等硬件配置与你的环境一致
