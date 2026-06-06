# 新机器快速恢复指南

> 从 GitHub 克隆 SEU_MD_PX4 项目并恢复到可编译状态

---

## 1. 系统要求

- **OS**: Ubuntu 20.04 LTS (Focal Fossa)
- **磁盘空间**: 至少 10 GB 空闲（项目约 3GB + build 缓存）
- **内存**: 建议 8 GB+
- **网络**: 需要访问 GitHub 以下载代码和子模块

---

## 2. 安装系统依赖

```bash
sudo apt update && sudo apt upgrade -y

# PX4 基础依赖
sudo apt install -y \
    git wget cmake build-essential \
    python3 python3-pip python3-dev \
    ninja-build libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgl1-mesa-dev libxrandr-dev libxinerama-dev libxcursor-dev \
    libxi-dev libglu1-mesa-dev libtool autoconf automake \
    pkg-config python3-jinja2 python3-empy python3-toml \
    python3-numpy python3-yaml \
    libSDL2-dev libopencv-dev \
    g++-arm-linux-gnueabihf gcc-arm-linux-gnueabihf \
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    gdb-multiarch

# 安装 PX4 Python 工具包
pip3 install --user -U pyserial pymavlink pyulog \
    empy pyyaml jinja2 toml numpy pandas

# 添加用户到 dialout 组（串口权限）
sudo usermod -aG dialout $USER
# 重新登录或执行：
newgrp dialout
```

---

## 3. 克隆项目

```bash
# 选择工作目录
cd ~

# 克隆主仓库（--recursive 必须！否则子模块缺失会导致编译失败）
git clone --recursive https://github.com/MYAknight/SEU_MD_PX4.git

# 进入项目目录
cd SEU_MD_PX4

# 如果克隆时子模块网络中断，补全子模块
git submodule update --init --recursive
```

> ⚠️ **关键警告**：PX4 有 17 个子模块（NuttX 操作系统、Gazebo 仿真模型等），`--recursive` 是必须的。如果漏掉，编译会报找不到 `NuttX` 或 `matrix` 库等错误。

---

## 4. 项目结构确认

克隆完成后，确认关键目录存在：

```bash
ls -d src/modules/huaqiccc_morph_control          # 变形控制模块
ls -d ROMFS/px4fmu_common/init.d/airframes/4401*  # 实机机架配置
ls -d Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/huaqiccc  # SITL 模型
ls platforms/nuttx/NuttX                          # RTOS（子模块）
```

---

## 5. 编译验证

### 5.1 实机固件（Pixhawk 6C）

```bash
cd ~/SEU_MD_PX4
make px4_fmu-v6c_default -j$(nproc)
```

成功后会生成：
```
build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

### 5.2 SITL 仿真固件（可选）

```bash
make px4_sitl_default -j$(nproc)
```

---

## 6. 连接飞控与上传固件

### USB 连接飞控

```bash
# 查看设备
ls /dev/ttyACM*

# 连接 nsh 终端诊断
python3 Tools/mavlink_shell.py /dev/ttyACM0 --baud 57600
```

### 上传固件到飞控

```bash
cd ~/SEU_MD_PX4

# 1. 编译
make px4_fmu-v6c_default -j$(nproc)

# 2. 重启飞控到 bootloader
python3 Tools/mavlink_shell.py /dev/ttyACM0 --baud 57600 <<< 'reboot -b'

# 3. 等待 3 秒，然后上传
sleep 3
python3 Tools/px_uploader.py \
    --port /dev/ttyACM0 \
    build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

---

## 7. 安装 QGroundControl（地面站）

```bash
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage

# 运行
./QGroundControl.AppImage
```

首次运行需要添加用户到 `plugdev` 组：
```bash
sudo usermod -aG plugdev $USER
```

---

## 8. 常见问题

### Q: 克隆后编译报错 `NuttX not found`
```bash
# 原因：子模块未拉取
git submodule update --init --recursive
```

### Q: `make px4_fmu-v6c_default` 报错 `command not found`
```bash
# 原因：依赖未安装完整
bash ./Tools/setup/ubuntu.sh
```

### Q: 串口权限 denied
```bash
# 原因：用户不在 dialout 组
sudo usermod -aG dialout $USER
# 注销并重新登录
```

### Q: 上传固件时 `Waiting for bootloader...` 超时
```bash
# 原因：飞控未进入 bootloader
# 先执行 reboot -b，等待设备重新出现后再上传
```

### Q: 编译时内存不足
```bash
# 减少并行任务数
make px4_fmu-v6c_default -j2
```

---

## 9. 项目关键配置速查

| 配置项 | 值 | 说明 |
|--------|-----|------|
| 机架 | `4401_huaqiccc_real` | 实机配置 |
| 变形模块 | `huaqiccc_morph_control` | AS5600 + AUX1 舵机 |
| 电机 PWM | MAIN1-4 | 前左/后左/前右/后右 |
| 变形 PWM | AUX1 | 1000-2000μs，REV=1 |
| 高度参考 | EKF2_HGT_REF=3 | Vision (MoCap) |
| GPS | EKF2_GPS_CTRL=0 | 禁用 |
| 磁罗盘 | EKF2_MAG_TYPE=5 | None |

---

## 10. 联系我们

- GitHub: https://github.com/MYAknight/SEU_MD_PX4
- 项目基于: PX4 v1.14.3
