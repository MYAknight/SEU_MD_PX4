# Agent 自动化能力记录

## 1. 固件编译与上传（已验证）

### 能力说明
Agent 可以直接在当前 Ubuntu 环境中完成 PX4 固件的**全自动编译+上传**，无需用户手动操作 QGroundControl。

### 工作流程
```
用户提出需求 → Agent 修改源码 → make 编译 → px_uploader.py 自动上传 → 飞控重启
```

### 具体命令
```bash
# 编译
cd /home/a/PX4-Autopilot
make px4_fmu-v6c_default -j$(nproc)

# 上传（需先进入 bootloader）
python3 Tools/mavlink_shell.py /dev/ttyACM0 --baud 57600 <<< 'reboot -b'
python3 Tools/px_uploader.py --port /dev/ttyACM0 build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

### 注意事项
- 上传前必须先发送 `reboot -b` 让飞控进入 bootloader 模式
- 仅支持 USB 直连 (`/dev/ttyACM*`)，数传 (`/dev/ttyUSB*`) 因 VMware+CH340 兼容性问题不稳定
- 编译产物路径：`build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`

### 已验证的上传记录
- **2026-06-06**: 成功上传 CT 精度修复固件（删除 airframe CT 设置 + 默认值 6.5）

---

## 2. 飞控实时诊断（已验证）

### 能力说明
通过 USB 直连，Agent 可以实时连接 nsh 终端，收集参数、运行测试命令、检查模块状态。

### 具体命令
```bash
# 连接 nsh 并批量执行诊断
python3 Tools/mavlink_shell.py /dev/ttyACM0 --baud 57600 <<< $'ver all\nparam show CA_ROTOR*\ncontrol_allocator status\nps\n'

# 电机测试
python3 Tools/mavlink_shell.py /dev/ttyACM0 --baud 57600 <<< $'actuator_test set -m 1 -v 0.1 -t 3\n'
```

### 限制
- 需要飞控通过 USB 连接到当前 Ubuntu 虚拟机
- 数传连接因 CH340 I/O 错误不稳定（VMware 环境问题）

---

## 3. 无法做到的事情

| 事项 | 原因 |
|------|------|
直接访问数传串口 | VMware + CH340 兼容性问题，/dev/ttyUSB0 频繁 I/O error |
操作 QGroundControl | 无图形界面访问能力 |
物理操作飞控（按键等）| 无机械臂/物理接口 |
直接读取 AS5600 编码器 | 无 I2C 硬件访问能力 |
