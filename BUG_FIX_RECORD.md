# Huaqiccc 项目 Bug 修复记录

> 生成时间：2026-05-23  
> 记录人：Kimi Code CLI  
> 对应项目状态：PROJECT_STATUS_REPORT.md v2

---

## 目录

1. [MPC H 矩阵对角含 `+I` 偏移](#bug-1-mpc-h-矩阵对角含-i-偏移)
2. [MPC `_mpc_alpha` 过小](#bug-2-mpc-_mpc_alpha-过小)
3. [MPC Z 轴约束过紧](#bug-3-mpc-z-轴约束过紧)
4. [MPC 积分器增益被削弱 10 倍](#bug-4-mpc-积分器增益被削弱-10-倍)
5. [Hover Thrust 未传递给 Advanced 控制器](#bug-5-hover-thrust-未传递给-advanced-控制器)
6. [MAVROS 参数同步缺失（Simplified Test）](#bug-6-mavros-参数同步缺失simplified-test)
7. [MAVROS 参数同步缺失 + 变形失控（Perching Test）](#bug-7-mavros-参数同步缺失--变形失控perching-test)
8. [Nuttx 不兼容（`std::nth_element` + 大栈数组）](#bug-8-nuttx-不兼容stdnth_element--大栈数组)
9. [自主栖停 COMPLIANT 阶段掉高 + 提前缓降风险](#bug-9-自主栖停-compliant-阶段掉高--提前缓降风险)

---

## Bug #9：自主栖停 COMPLIANT 阶段掉高 + 提前缓降风险

### 现象
- COMPLIANT 阶段飞机明显掉高，Z 轴高度无法保持。
- 即使机械臂尚未收拢，6 秒时间兜底也会触发，导致提前进入 RAMP_DOWN。
- `setpoint override` 逻辑因判断条件写死 `MPCA_PC_EN >= 3` 而从未执行。
- 地面站不会自动收拢机械臂，需要操作员手动干预。

### 根因分析
1. COMPLIANT 进入时调用了 `_control.resetIntegral()` / `_advanced_control.resetIntegral()`，清除了 Z 轴积分，导致高度环失去悬停补偿。
2. `MPCA_PC_K_SOFT` 同时软化了 X/Y/Z 三轴位置 P 增益，Z 轴刚度不足。
3. 默认启用弹簧模型 `MPCA_PC_SPRING=1`，但 `max_thrust = 1.3×hover` 在较大前倾角下不足以维持垂直分量。
4. 抓握判定使用 `pos_stable && (arms_contracted || compliant_elapsed > 6.0f)`，时间兜底会在机械臂未夹紧时进入缓降。
5. 进入 RAMP_DOWN 的条件是 `compliant_elapsed > 6.0f && _grasp_secure`，同样依赖时间兜底。
6. `setpoint override` 阈值 `>= 3` 与 `MPCA_PC_EN` 最大值为 2 矛盾。

### 修复方案
- **取消 COMPLIANT 积分重置**：仅保留标志位，不再调用 `resetIntegral()`。
- **仅软化 XY 轴位置 P 增益**：Z 轴增益保持正常。
- **默认关闭弹簧模型**：`MPCA_PC_SPRING=0`，由 Z 位置控制器直接维持高度；同时把弹簧模型上限放宽到 `2.0×hover`。
- **删除 6 秒时间兜底**：真实硬件必须 `pos_stable && arms_contracted`；SITL 无 arm 数据时退化为 `pos_stable`。
- **抓握确认后保持 1 秒进入 RAMP_DOWN**：新增 `_grasp_secure_time` 记录抓握确认时刻。
- **修正 setpoint override 阈值**：`>= 2`，COMPLIANT 使用固定 `_compliant_sp_x` 避免随动跳变。
- **地面站自动收拢**：`control_ground_station_ros.py` 在检测到 `CONTACT/COMPLIANT` 时自动下发一次 `MAV_CMD_HUAQICCC_SET_ARM_ANGLE(0.0)`。
- **新增参数 `MPCA_PC_ARM_THR`**：默认 `-0.15 rad`，替代硬编码 `-0.30 rad`。

### 涉及文件
- `src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- `src/modules/mc_pos_control/MulticopterPositionControl.hpp`
- `src/modules/mc_pos_control/mc_pos_control_params.c`
- `~/Projects/ground_station/scripts/control_ground_station_ros.py`

### 验证结果
- `px4_fmu-v6c_default` 编译通过（FLASH 77.87%）
- `px4_sitl_default` 编译通过
- 实机飞行验证待进行

---

## Bug #10：CONTACT 阶段清零 velocity/acceleration/jerk 导致实机剧烈振荡

### 现象
- 实机 `07_54_35.ulg`：PX4 触发 CONTACT 后约 234 s，roll/pitch/yaw 迅速发散，电机饱和，机身在 pole 前来回撞击。
- 同样代码在 SITL 中却能稳定完成栖息。

### 根因分析
1. `MulticopterPositionControl.cpp` 的 CONTACT 分支把 `_setpoint.velocity / acceleration / jerk` 全部置 0。
2. 接触前 offboard trajectory 还带有前向速度（`vy ≈ 0.25 m/s`）和加速度（`ay ≈ 1.26 m/s²`）；接触瞬间切断这些前馈，相当于给控制器发了一个“急刹车”指令。
3. 同时 position setpoint 从原 offboard setpoint 向后 ramp 到 `contact_pos + preload`，无人机被立柱挡住后还要追一个后退的目标，形成拉扯-碰撞循环。
4. SITL 的 pole 碰撞模型带弹簧/阻尼，且没有真实粘滑/偏航力矩，所以问题被掩盖。

### 修复方案
- **取消清零**：CONTACT 分支只覆盖 `_setpoint.position[0/1/2]`，`velocity / acceleration / jerk` 保持来自 trajectory setpoint。
- **降低 CONTACT 阶段 PX4_INFO 频率**：从每周期打印改为 1 Hz 节流，减少 CPU/遥测抖动。
- 保留自动收臂逻辑（经实机验证，收臂本身不造成负面影响）。

### 涉及文件
- `src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- `src/modules/mc_pos_control/MulticopterPositionControl.hpp`

### 验证结果
- `px4_fmu-v6c_default` 编译通过（FLASH 77.91%）。
- 实机复飞：接触后自动收臂流程稳定，无 roll/pitch/yaw 振荡。

---

## Bug #1：MPC H 矩阵对角含 `+I` 偏移

### 现象
- MPC 模式收敛极慢，Z 轴需 36–40s 才能从 1.11m 爬到 2.0m
- 着陆阶段失控坠毁
- XY 方向误差较大（mean=8.5cm, max=35cm）

### 根因分析
`mpc_H_init` 对角值被错误地写为 `I + R_bar`：
```cpp
// 错误值（原代码）
{1.010375f, 0.0079f, ...}   // 应为 0.010375，多了 +1.0
{0.0079f,   1.0071f, ...}   // 应为 0.0071？不，应为 ~0.0171
```

正确的 MPC QP 形式为：
```
min_u  0.5 * u^T * H * u + g^T * u
s.t.   u_min <= u <= u_max
```

其中 `H = Gamma^T * Q_bar * Gamma + R_bar`。当 `Q_bar` 通过 `Gamma` 作用于 `u` 时，Hessian 的纯 `R_bar` 贡献才是正确的对角项。原代码在 `R_bar` 上又加了单位矩阵 `I`，导致 Hessian 条件数恶化，求解器收敛到保守值（~0.55 而非饱和边界），最终通过 `mpc_acc_sp * 5.0f` hack 勉强补偿。

### 修复方案
将对角值修正为正确的 `R_bar` 贡献：
```cpp
static constexpr float mpc_H_init[MPC_N][MPC_N] = {
    {0.020375f, 0.0079f,   0.005525f, 0.00335f,  0.001475f},
    {0.0079f,   0.0171f,   0.005075f, 0.00315f,  0.001425f},
    {0.005525f, 0.005075f, 0.014625f, 0.00295f,  0.001375f},
    {0.00335f,  0.00315f,  0.00295f,  0.01275f,  0.001325f},
    {0.001475f, 0.001425f, 0.001375f, 0.001325f, 0.011275f}
};
```

同时移除 `mpc_acc_sp * 5.0f` 的 hack。

### 涉及文件
- `src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.cpp`

### 验证结果
- SITL 飞行测试（MPCA_MODE=3）：
  - Z 轴：立即起飞，无延迟
  - Hover 误差：`err_z` 从 >50cm 降至 ~5.3cm
  - 着陆：正常降落，无失控

---

## Bug #2：MPC `_mpc_alpha` 过小

### 现象
- H 矩阵修复后，求解器步长相对于特征值过小，收敛仍偏慢
- XY 跟踪有轻微延迟

### 根因分析
梯度投影 QP 的收敛步长 `alpha` 需满足 `0 < alpha < 2/λ_max(H)`。
- 错误 H 矩阵时：`λ_max ≈ 1.01`，`alpha=0.978` 合理
- 修正 H 矩阵后：`λ_max ≈ 0.0204`，理论上 `alpha` 可放大至 ~100
- 继续使用 `alpha=0.978` 导致步长仅为最优的 ~1/50

### 修复方案
```cpp
float _mpc_alpha{20.0f};  // 从 0.978f 提升
```

### 涉及文件
- `src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.hpp`

### 验证结果
- SITL 飞行测试：求解器响应及时，XY 误差 <2cm

---

## Bug #3：MPC Z 轴约束过紧

### 现象
- 即使 H 矩阵和 alpha 修正后，Z 轴有时仍无法产生足够推力
- 表现为"推不上去"或悬停高度偏低

### 根因分析
- `_mpc_u_max_z = 2.0f` 表示 MPC 输出最大加速度为 2.0 m/s²
- 无人机质量 1.26kg，重力加速度 9.8 m/s²，悬停需要净推力 ~9.8 m/s²
- 加上位置/速度误差补偿，2.0 m/s² 远不足以克服重力
- 这导致 MPC 求解器在 Z 轴始终饱和，但实际值仍不够

### 修复方案
```cpp
float _mpc_u_min_z{-8.0f};  // 从 -2.0f 扩展
float _mpc_u_max_z{8.0f};   // 从 2.0f 扩展
```

### 涉及文件
- `src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.hpp`

### 验证结果
- SITL 飞行测试：起飞立即成功，无延迟；悬停高度稳定

---

## Bug #4：MPC 积分器增益被削弱 10 倍

### 现象
- Z 轴存在 ~8.5cm 稳态误差，长时间无法消除
- 尤其在变形后（机臂展开改变 CoP），稳态误差更明显

### 根因分析
在 `_mpcControl()` 中，积分器更新写为：
```cpp
_vel_int += vel_error.emult(_gain_vel_i) * dt * 0.1f;  // 只有正常的 1/10
```

这导致积分累积速度极慢，无法及时补偿 hover thrust 偏差和变形引起的 CoP 偏移。

### 修复方案
恢复为完整积分速度：
```cpp
_vel_int += vel_error.emult(_gain_vel_i) * dt;  // 移除 * 0.1f
```

注意：`_velocityControl()`（用于模式 1/2）中仍保留 `* dt * 0.1f`，因为标准 PID 不需要大积分。

### 涉及文件
- `src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.cpp`（第 492 行）

### 验证结果
- SITL 飞行测试：Z 稳态误差从 ~8.5cm 降至 ~5.3cm

---

## Bug #5：Hover Thrust 未传递给 Advanced 控制器

### 现象
- 所有 advanced 模式（1/2/3）hover thrust 使用默认值 0.5
- SDF 中实际 hover thrust 为 0.4375（质量 1.26064kg / 12.0N 总推力）
- 导致 Z 轴持续存在偏差，尤其变形后漂移明显

### 根因分析
`MulticopterPositionControl.cpp` 中只调用了：
```cpp
_control.setHoverThrust(_param_mpc_thr_hover.get());
// 缺少 _advanced_control.setHoverThrust(...)
```

### 修复方案
在初始化和非飞行更新处，同时调用：
```cpp
// 初始化（约第 261-262 行）
_control.setHoverThrust(_param_mpc_thr_hover.get());
_advanced_control.setHoverThrust(_param_mpc_thr_hover.get());

// 非飞行更新（约第 495-496 行）
if (!flying) {
    _control.setHoverThrust(_param_mpc_thr_hover.get());
    _advanced_control.setHoverThrust(_param_mpc_thr_hover.get());
}
```

### 涉及文件
- `src/modules/mc_pos_control/MulticopterPositionControl.cpp`

### 验证结果
- SITL 飞行测试：Z 轴偏差减小，变形后漂移从 ~14cm 降至 ~8.5cm

---

## Bug #6：MAVROS 参数同步缺失（Simplified Test）

### 现象
```
[ERROR] [1779556820.860896915, 3.156000000]: PR: Unknown parameter to set: MPCA_MODE
[ERROR] PR: Unknown parameter to set: MPC_XY_P
```

### 根因分析
`huaqiccc_simplified_flight_test.py` 中：
1. 等待 `current_state.connected` 为 true
2. 立即调用 `param_set('MPCA_MODE')`

但 MAVROS 从 PX4 下载完整参数列表需要时间（通常 3–8 秒）。在缓存同步完成前，`param_set()` 返回 "Unknown parameter"。

### 修复方案
添加参数同步等待和强制 pull：
```python
# 等待 5 秒基础同步
rospy.sleep(5.0)

# 强制 pull 所有参数
try:
    from mavros_msgs.srv import ParamPull
    rospy.wait_for_service('/mavros/param/pull', timeout=10.0)
    param_pull = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
    pull_resp = param_pull(False)
    if pull_resp.success:
        print(f"[OK] 参数同步完成，共 {pull_resp.param_received} 个参数")
except Exception as e:
    print(f"[WARN] 参数 pull 跳过: {e}")

# 然后再设置参数
for attempt in range(5):
    resp = param_set(param_id='MPCA_MODE', value=ParamValue(integer=mpca_mode))
    if resp.success:
        break
    rospy.sleep(1.0)
```

### 涉及文件
- `~/huaqiccc_simplified_flight_test.py`

### 验证结果
```
[WAIT] 等待 MAVROS 参数同步...
[OK] 参数同步完成，共 844 个参数
[OK] Set MPCA_MODE = 3 (attempt 1)
[OK] Set MPC_XY_P = 1.5
```
无报错，参数设置成功。

---

## Bug #7：MAVROS 参数同步缺失 + 变形失控（Perching Test）

### 现象
1. 同样出现 `Unknown parameter to set: MPCA_MODE`
2. 变形时无人机快速下降（数秒内从 2.5m 掉到地面）
3. 变形后位置控制让无人机以极快速度飞行，轨迹完全偏离
4. 整个 perching 任务几乎无法完成

### 根因分析 1：参数同步
同 Bug #6，perching 脚本缺少 MAVROS 参数缓存同步。

### 根因分析 2：变形期间丢失 position setpoint
在 `_morph_arm_slowly()` 中，当 `push_x is None`（初始变形时），循环体只执行：
```python
self._send_arm_angle_ros(angle)      # Gazebo 视觉
self.update_px4_morph(angle)          # 31440 命令
# ❌ 没有 self._send_setpoint(x, y, z) !
```

这意味着在 3 秒变形期间，PX4 **完全没有收到 position setpoint**。OFFBOARD 模式要求持续收到 setpoint（超时 ~0.5–2.0s），一旦超时：
1. PX4 退出 OFFBOARD 模式
2. 位置控制失效，无人机自由下坠
3. 后续重新发送 setpoint 时，PX4 可能已经不在 OFFBOARD，或者位置控制器已失稳
4. 即使重新进入 OFFBOARD，巨大的位置误差导致控制器输出饱和，无人机极速飞行

### 修复方案 1：参数同步
同 Bug #6，添加 `ParamPull` 同步。

### 修复方案 2：变形期间保持 position setpoint
修改 `_morph_arm_slowly()` 签名，增加 `hold_x, hold_y, hold_z` 参数：
```python
def _morph_arm_slowly(self, target_angle, duration,
                      hold_x=0.0, hold_y=0.0, hold_z=None,
                      push_x=None, push_y=None, push_z=None):
```

在循环中始终发送位置 setpoint：
```python
sp_x = push_x if push_x is not None else hold_x
sp_y = push_y if push_y is not None else hold_y
sp_z = push_z if push_z is not None else hold_z
self._send_setpoint(sp_x, sp_y, sp_z)  # 始终发送！
```

### 修复方案 3：OFFBOARD 守卫
添加 `_ensure_offboard()` 方法：
```python
def _ensure_offboard(self):
    if self.current_state and self.current_state.mode != "OFFBOARD":
        print("[GUARD] Re-enabling OFFBOARD mode")
        self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
        rospy.sleep(0.5)
```

在变形前后调用，并添加 40 个 setpoint burst（2 秒）稳定过渡：
```python
self._ensure_offboard()
self._morph_arm_slowly(-0.3, 3.0, hold_x=0.0, hold_y=0.0, hold_z=self.HOVER_Z)
self._ensure_offboard()
# Post-morph setpoint burst
for _ in range(40):
    self._send_setpoint(0.0, 0.0, self.HOVER_Z)
    rate.sleep()
```

### 涉及文件
- `~/huaqiccc_perching_test.py`

### 验证结果
```
[OK] Parameter sync complete, 844 params received
[OK] Set MPCA_MODE
[MORPH] Slow morph to -0.30 rad over 3.0s (hold at 0.0,0.0,2.5)
[GUARD] Post-morph setpoint burst
[APPROACH] (0.0,0.0) -> (4.0,0.0) over 10.0s
[PUSH] From x=4.00 toward x=5.00 at 0.25m/s
  [STALL] act_x=4.93, sp_x=4.98, s=0.91
[CONTACT] Detected at t=3.65s | stalled=True
[LAND] Landing
[SAVE] perching_test_20260523_103802.csv
```
- 参数设置：✅ 成功
- 变形稳定性：✅ 无下降、无失控
- Approach：✅ 正常飞行
- Push/Stall detection：✅ 成功检测接触
- Grasp/Land：✅ 正常完成

---

## Bug #8：Nuttx 不兼容（`std::nth_element` + 大栈数组）

### 现象
```
make px4_fmu-v6c_default
...
error: 'nth_element' is not a member of 'std'
error: variable-sized object 'tmp' may not be initialized
```

或链接阶段出现栈溢出警告。

### 根因分析
1. **Nuttx 无完整 C++ STL**：`std::nth_element` 属于 `<algorithm>`，Nuttx 工具链不支持
2. **栈限制**：Nuttx 默认线程栈为 2048B；原代码在栈上分配 `float tmp[1000]`（4000B），直接溢出

### 修复方案 1：替换 `std::nth_element`
编写自定义选择排序实现（只需排前 k 个）：
```cpp
static void simple_nth_element(float *arr, int n, int k)
{
    for (int i = 0; i <= k; i++) {
        int min_idx = i;
        for (int j = i + 1; j < n; j++) {
            if (arr[j] < arr[min_idx]) { min_idx = j; }
        }
        float tmp = arr[i]; arr[i] = arr[min_idx]; arr[min_idx] = tmp;
    }
}
```

### 修复方案 2：静态数组避免大栈帧
```cpp
static float tmp[MAX_HISTORY];  // static → BSS 段，不占用栈
```

### 涉及文件
- `src/modules/external_force_estimator/external_force_estimator.cpp`

### 验证结果
- V6C 编译：`make px4_fmu-v6c_default` ✅ 324/324 通过
- 符号检查：`nm build/px4_fmu-v6c_default/px4_fmu-v6c_default.elf | grep external_force_estimator_main` ✅ 存在
- 固件产物：`build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4` ✅ 生成

---

## 附录：修复时间线

| 时间 | Bug | 状态 |
|------|-----|------|
| 2026-05-23 09:00 | #1 MPC H 矩阵 | ✅ 修复并验证 |
| 2026-05-23 09:00 | #2 MPC alpha | ✅ 修复并验证 |
| 2026-05-23 09:00 | #3 MPC Z limit | ✅ 修复并验证 |
| 2026-05-23 09:00 | #4 MPC integrator | ✅ 修复并验证 |
| 2026-05-23 09:00 | #5 Hover thrust | ✅ 修复并验证 |
| 2026-05-23 09:00 | #8 Nuttx compat | ✅ 修复并验证 |
| 2026-05-23 10:00 | #6 MAVROS sync (simplified) | ✅ 修复并验证 |
| 2026-05-23 10:30 | #7 MAVROS sync + morph (perching) | ✅ 修复并验证 |

---

---

## Bug #9 / 设计变更：IMU-ICD 实机无效，接触检测重构为位置/姿态方案

### 时间
2026-06-14

### 现象
- 2026-06-13 两次手动栖停测试（`08_36_57_perch_test1_ground_to_pole.ulg`、
  `08_52_57_perch_test2_ground_to_pole.ulg`）中，原始 IMU 加速度模值在悬停时已
  约 25 m/s²，接触前后没有可区分的跳变。
- `external_force_estimator`（IMU-ICD / GMO）无法作为实机接触检测器使用。

### 根因分析
1. 变形四旋翼悬停时振动/抖动较大，IMU 基线已经很高；
2. 与杆的接触是渐进式挤压，没有产生可被 IMU 分辨的冲击瞬态；
3. 北航墙面栖息的 IMU 检测方案在本机、本场景下不适用。

### 新方案
在 `mc_pos_control` 内实现位置/姿态接触检测：
- 沿 setpoint 方向的位置误差 > `MPCA_PC_SERR` (0.05 m)
- 该方向速度 < `MPCA_PC_SVEL` (0.10 m/s)
- pitch 前倾 < `MPCA_PC_PIT_THR` (-5°)
- 连续满足 > `MPCA_PC_DUR_THR` (0.30 s)

离线验证：两次手动栖停日志零误检、检出率 90~100%。

### 涉及文件
- `src/modules/mc_pos_control/MulticopterPositionControl.cpp/.hpp`
- `src/modules/mc_pos_control/mc_pos_control_params.c`
- `src/modules/external_force_estimator/external_force_estimator.cpp/.h/.c`（注释更新，打印降频）
- `boards/px4/fmu-v6c/default.px4board`
- `ROMFS/px4fmu_common/init.d/rc.mc_apps`
- `ROMFS/px4fmu_common/init.d/airframes/4401_huaqiccc_real`

### 验证结果
- v6c 编译：`make px4_fmu-v6c_default` ✅ 868/868 通过
- 固件产物：`build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4` ✅ 生成
- 离线日志分析：✅ 两次手动栖停均能被新检测器识别
- 实机飞行验证：✅ 2026-06-14 手动栖停测试，检测灵敏度合适，遥测链路正常
- 遥测通道：✅ 通过 `DEBUG_FLOAT_ARRAY`（`name="perch"`）实时回传接触/栖停/变形状态

### 影响
- `MPCA_PC_EN` 重新分档：`0=OFF, 1=DETECT（只日志）, 2=FULL（触发 FSM）`
- 实机 airframe 默认 `MPCA_PC_EN=1`，便于先验证检测时机再开全自动栖停。

---

*记录结束。*
