# 核心代码片段

> 本文件收录项目中最关键、最可能被后续修改或复用的代码片段。
> 按模块分类，包含文件路径、功能说明和完整代码。

---

## 1. 微分平坦前馈（FlatnessFeedforward）

**文件**：`src/modules/mc_pos_control/FlatnessFeedforward/FlatnessFeedforward.hpp`

```cpp
#pragma once

#include <matrix/matrix/math.hpp>

using namespace matrix;

class FlatnessFeedforward {
public:
    struct FlatOutput {
        Vector3f pos, vel, acc, jerk, snap;
        float yaw, yaw_dot, yaw_ddot;
        float arm_angle, arm_angle_dot, arm_angle_ddot;
    };
    struct Feedforward {
        float collective_thrust;
        Vector3f body_z;
        Matrix3f R;
        Vector3f angular_velocity;
        Vector3f angular_acceleration;
        float servo_angle, servo_rate;
    };
    bool compute(const FlatOutput& xi, float mass,
                 const SquareMatrix3f& J, float thrust_k,
                 Feedforward& out);
    bool computeSimple(const FlatOutput& xi, float mass, Feedforward& out);
private:
    float _g{9.80665f};
    bool _computeAttitude(const Vector3f& z_b, float yaw, Matrix3f& R) const;
    bool _computeAngularVelocity(const Vector3f& z_b, const Vector3f& acc,
                                 const Vector3f& jerk, float thrust, float mass,
                                 float yaw_dot, const Matrix3f& R, Vector3f& omega) const;
    bool _computeAngularAcceleration(const Vector3f& z_b, const Vector3f& acc,
                                     const Vector3f& jerk, const Vector3f& snap,
                                     float thrust, float thrust_dot, float mass,
                                     float yaw_dot, float yaw_ddot,
                                     const Matrix3f& R, const Vector3f& omega,
                                     Vector3f& alpha) const;
};
```

**关键设计**：
- `compute()`：完整flatness映射（含角加速度）
- `computeSimple()`：仅位置级（acc→attitude+thrust），计算量更小
- 输入包含arm_angle及其导数，支持变形无人机扩展

---

## 2. MPC+前馈集成（AdvancedPositionControl::update）

**文件**：`src/modules/mc_pos_control/AdvancedPositionControl/AdvancedPositionControl.cpp`

```cpp
} else if (_mode == 3) {
    // 预计算flatness加速度前馈（在MPC之前）
    _acc_sp_ff.zero();
    _ang_vel_ff.zero();
    _ang_acc_ff.zero();

    if (_use_flatness_ff && _flat_output_valid) {
        FlatnessFeedforward::Feedforward ff;
        // 从LUT插值获取惯量
        float J_vals[3] = {_J_lut[0][0], _J_lut[0][1], _J_lut[0][2]};
        if (_J_lut_initialized) {
            float abs_angle = fabsf(_arm_angle);
            float idx_f = abs_angle / _gs_lut_step;
            int idx_low = static_cast<int>(floorf(idx_f));
            int idx_high = idx_low + 1;
            float w_high = idx_f - static_cast<float>(idx_low);
            if (idx_low < 0) { idx_low = 0; }
            if (idx_high >= GS_LUT_SIZE) { idx_high = GS_LUT_SIZE - 1; }
            if (idx_low >= GS_LUT_SIZE) { idx_low = GS_LUT_SIZE - 1; }
            for (int j = 0; j < 3; ++j) {
                J_vals[j] = _J_lut[idx_low][j] * (1.0f - w_high) + _J_lut[idx_high][j] * w_high;
            }
        }
        SquareMatrix3f J;
        J.setZero();
        J(0, 0) = J_vals[0];
        J(1, 1) = J_vals[1];
        J(2, 2) = J_vals[2];

        if (_flatness_ff.compute(_flat_output, _vehicle_mass, J, 0.0f, ff)) {
            // 前馈以加速度[m/s²]形式计算
            _acc_sp_ff = Vector3f(_flat_output.acc) * _flatness_blend;
            _ang_vel_ff = ff.angular_velocity * _flatness_blend;
            _ang_acc_ff = ff.angular_acceleration * _flatness_blend;
        }
    }

    _positionControl();
    _mpcControl(dt);
    // _acc_sp_ff在_mpcControl内部被添加到_acc_sp
}
```

**关键设计**：
- `_acc_sp_ff`以加速度形式（m/s²）计算，而非直接推力（N）
- 惯量J通过LUT插值，支持变形过程中的时变惯量
- `_flatness_blend`（默认0.3）用于抑制噪声放大

---

## 3. 控制效率矩阵LUT更新

**文件**：`src/modules/control_allocator/huaqiccc_motor_lut.hpp`

```cpp
#pragma once

struct HuaqicccMotorParams {
    float px[4];  // motor 0=lb, 1=lf, 2=rb, 3=rf
    float py[4];
    float pz[4];
};

static constexpr int HUAQICCC_LUT_SIZE = 11;
static constexpr float HUAQICCC_LUT_STEP = 0.05f;
static constexpr float HUAQICCC_LUT_MAX_ANGLE = -0.5f;

static constexpr HuaqicccMotorParams huaqiccc_motor_lut[HUAQICCC_LUT_SIZE] = {
    {   // index=0, angle=0.00 rad
        { -0.18773f, +0.27137f, -0.19233f, +0.26727f },  // px
        { -0.24887f, -0.22907f, +0.24553f, +0.23303f },  // py
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }   // pz
    },
    // ... index=1~10 ...
};

static inline void huaqiccc_get_motor_params(float arm_angle, float px[4], float py[4], float pz[4]) {
    if (arm_angle > 0.0f) arm_angle = 0.0f;
    if (arm_angle < HUAQICCC_LUT_MAX_ANGLE) arm_angle = HUAQICCC_LUT_MAX_ANGLE;
    int idx = static_cast<int>(roundf(fabsf(arm_angle) / HUAQICCC_LUT_STEP));
    if (idx < 0) idx = 0;
    if (idx >= HUAQICCC_LUT_SIZE) idx = HUAQICCC_LUT_SIZE - 1;
    for (int i = 0; i < 4; ++i) {
        px[i] = huaqiccc_motor_lut[idx].px[i];
        py[i] = huaqiccc_motor_lut[idx].py[i];
        pz[i] = huaqiccc_motor_lut[idx].pz[i];
    }
}
```

**文件**：`src/modules/control_allocator/ControlAllocator.cpp`（关键片段）

```cpp
void ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
    static uORB::Subscription huaqiccc_morph_sub{ORB_ID(huaqiccc_morph_angle)};
    static float huaqiccc_arm_angle = 0.0f;
    static bool huaqiccc_active = false;

    huaqiccc_morph_angle_s morph_msg;
    if (huaqiccc_morph_sub.update(&morph_msg)) {
        huaqiccc_arm_angle = morph_msg.arm_angle;
        huaqiccc_active = true;
    }

    if (huaqiccc_active) {
        float px[4], py[4], pz[4];
        huaqiccc_get_motor_params(huaqiccc_arm_angle, px, py, pz);

        matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> eff_matrix;
        eff_matrix.setZero();
        const float thrust_z = -1.0f;
        const float km[4] = {-0.05f, 0.05f, 0.05f, -0.05f};

        for (int i = 0; i < 4; ++i) {
            eff_matrix(0, i) = py[i] * thrust_z;      // ROLL
            eff_matrix(1, i) = -px[i] * thrust_z;     // PITCH
            eff_matrix(2, i) = km[i];                  // YAW
            eff_matrix(5, i) = thrust_z;                // THRUST_Z
        }

        matrix::Vector<float, NUM_ACTUATORS> trim, lin_point;
        trim.setZero();
        lin_point.setZero();

        for (int i = 0; i < _num_control_allocation; ++i) {
            if (_control_allocation[i] != nullptr) {
                _control_allocation[i]->setEffectivenessMatrix(
                    eff_matrix, trim, lin_point, 4, true);
            }
        }
        _last_effectiveness_update = hrt_absolute_time();
        return; // 跳过标准更新路径
    }
    // ... 标准更新路径 ...
}
```

**关键设计**：
- LUT通过SDF运动学预计算，11点覆盖0至-0.50 rad
- 最近邻查找（非插值），满足1kHz实时性
- 原子更新所有control allocation实例

---

## 4. IMU-ICD接触检测FSM

**文件**：`src/modules/external_force_estimator/external_force_estimator.cpp`

```cpp
void ExternalForceEstimator::updateImuIcd(float a_mag, float a_hpf, float gyro_mag, float dt)
{
    float t = hrt_absolute_time() * 1e-6f;
    float impact_metric = a_mag + 2.0f * fabsf(a_hpf);  // 冲击度量
    _impact_metric_lpf = 0.9f * _impact_metric_lpf + 0.1f * impact_metric;

    float impact_thr = _param_force_thr.get();   // 默认2.0
    float t_thr = _param_time_thr.get();         // 默认0.03s
    float g_thr = _param_gyro_thr.get();         // 默认0.15 rad/s

    switch (_contact_state) {
    case STATE_NO_CONTACT:
        if (impact_metric > impact_thr) {
            _contact_start_time = t;
            _contact_state = STATE_IMPACT;
            _contact_confidence = 0.3f;
            _should_close = false;
        }
        break;

    case STATE_IMPACT:
        if (impact_metric < impact_thr * 0.3f) {
            _contact_state = STATE_NO_CONTACT;
            _contact_confidence = 0.0f;
            _should_close = false;
        } else if ((t - _contact_start_time) > t_thr) {
            _contact_state = STATE_CONFIRMED;
            _contact_confidence = 0.7f;
        }
        break;

    case STATE_CONFIRMED:
        if (gyro_mag < g_thr && impact_metric > impact_thr * 0.5f) {
            _contact_state = STATE_STABLE;
            _contact_confidence = 0.9f;
        } else if (impact_metric < impact_thr * 0.2f) {
            _contact_state = STATE_NO_CONTACT;
            _contact_confidence = 0.0f;
            _should_close = false;
        }
        break;

    case STATE_STABLE:
        if (gyro_mag > g_thr * 2.5f) {
            _contact_state = STATE_SLIPPING;
            _contact_confidence = 0.4f;
            _should_close = false;
        } else if (impact_metric < impact_thr * 0.15f) {
            _contact_state = STATE_NO_CONTACT;
            _contact_confidence = 0.0f;
            _should_close = false;
        } else if (_contact_confidence >= 0.85f) {
            _should_close = true;  // 触发夹持
        }
        break;

    case STATE_SLIPPING:
        if (gyro_mag < g_thr && impact_metric > impact_thr * 0.5f) {
            _contact_state = STATE_STABLE;
        } else if (impact_metric < impact_thr * 0.15f) {
            _contact_state = STATE_NO_CONTACT;
            _contact_confidence = 0.0f;
            _should_close = false;
        }
        break;
    }
}
```

**关键设计**：
- 冲击度量 = 加速度幅值 + 2×|高通滤波加速度|
- 5状态FSM：NO_CONTACT → IMPACT → CONFIRMED → STABLE → SLIPPING
- should_close在STABLE状态且置信度≥0.85时置位
- 参数可配置：impact_thr, time_thr, gyro_thr

---

## 5. Stall Detection（位置-速度堵转检测）

**文件**：`src/modules/mc_pos_control/MulticopterPositionControl.cpp`

```cpp
// === Position-based Stall Detection (drone blocked by obstacle) ===
bool stall_detected = false;
int pc_en = _param_mpca_pc_en.get();
int pc_trig = _param_mpca_pc_trig.get();
if (pc_en >= 2 && pc_trig != 2
    && _vehicle_control_mode.flag_control_offboard_enabled
    && flying) {
    // 使用Y轴（position[1]）与NED飞行轨迹一致
    // ⚠️ MAVROS将Python ENU坐标强制转为PX4 NED：Python X → NED Y
    float y_error = _setpoint.position[1] - states.position(1);
    float y_vel = states.velocity(1);

    bool approaching = y_error > _param_mpca_pc_stall_err.get();      // MPCA_PC_SERR=0.05
    bool nearly_stopped = fabsf(y_vel) < _param_mpca_pc_stall_vel.get();  // MPCA_PC_SVEL=0.10
    // 位置门限：可选，设为0.0则禁用，纯依赖局部运动判断
    bool near_pole_surface = true;
    float gate = _param_mpca_pc_gate.get();
    if (gate > 0.01f) {
        near_pole_surface = states.position(1) > gate;
    }

    // 时间-距离双重确认
    if (approaching && nearly_stopped && near_pole_surface) {
        if (_stall_start_time == 0) {
            _stall_start_time = hrt_absolute_time();
            _stall_start_y = states.position(1);
            if (_perching_phase == PerchingPhase::NONE) {
                mavlink_log_info(&_mavlink_log_pub, "STALL_START");
            }
        } else {
            float stall_elapsed = (hrt_absolute_time() - _stall_start_time) * 1e-6f;
            float dy = states.position(1) - _stall_start_y;
            if (stall_elapsed > _param_mpca_pc_stall_t.get()    // 默认1.0s
                && fabsf(dy) < _param_mpca_pc_stall_d.get()) {  // 默认0.03m
                stall_detected = true;
                if (_perching_phase == PerchingPhase::NONE) {
                    mavlink_log_info(&_mavlink_log_pub, "STALL_DETECTED");
                }
            }
        }
    } else {
        if (_stall_start_time != 0) {
            if (_perching_phase == PerchingPhase::NONE) {
                mavlink_log_info(&_mavlink_log_pub, "STALL_RESET");
            }
        }
        _stall_start_time = 0;
    }
}
```

**关键设计**：
- 纯位置/速度判断，无需知道柱子位置
- 三条件同时满足才启动计时：`approaching`（有目标在前）+ `nearly_stopped`（几乎停止）+ `near_pole_surface`（在门限区域内）
- 时间-距离双重确认：stall_t 秒内移动 < stall_d 才判定堵转
- 进入 perching 后停止打印日志，避免刷屏
- 与 IMU-ICD 并行，通过 `MPCA_PC_TRIG` 参数选择启用哪个

---

## 6. Perching触发逻辑（双触发源）

**文件**：`src/modules/mc_pos_control/MulticopterPositionControl.cpp`

```cpp
// === Perching Phase State Machine ===
// Auto-reset if perching disabled mid-flight
if (pc_en < 2 && _perching_phase != PerchingPhase::NONE) {
    _perching_phase = PerchingPhase::NONE;
    _stall_start_time = 0;
    _grasp_secure = false;
    _perching_active = false;
    _imu_stable_ever = false;
}

// 双触发源：Stall Detection 或 IMU Contact
if (pc_en >= 2 && _perching_phase == PerchingPhase::NONE
    && (stall_detected || (_perching_active && imu_contact_stable))) {
    _perching_phase = PerchingPhase::CONTACT;
    _perching_active = true;
    _perching_contact_x = states.position(1);
    _perching_start_z = states.position(2);
    _perching_start_time = hrt_absolute_time();
    _grasp_secure = false;
    _imu_stable_ever = false;
    mavlink_log_info(&_mavlink_log_pub, "Perching: contact detected, entering compliance");
}
```

**触发源选择**（`MPCA_PC_TRIG`）：
| 值 | 含义 | 效果 |
|----|------|------|
| 0 | BOTH | Stall + IMU 同时启用（任一触发） |
| 1 | STALL_ONLY | 仅 Stall Detection（IMU路径被 `pc_trig != 1` 禁用） |
| 2 | IMU_ONLY | 仅 IMU Contact（Stall路径被 `pc_trig != 2` 禁用） |

---

## 7. MAVROS加速度修复

---

## 6. MAVROS加速度修复

**文件**：`src/modules/mavlink/mavlink_receiver.cpp`

```cpp
// 在SET_POSITION_TARGET_LOCAL_NED消息处理中
setpoint.velocity[0] = (type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? (float)NAN : target_local_ned.vx;
setpoint.velocity[1] = (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? (float)NAN : target_local_ned.vy;
setpoint.velocity[2] = (type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? (float)NAN : target_local_ned.vz;

// === 关键修复：加速度字段必须写入setpoint.acceleration ===
setpoint.acceleration[0] = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? (float)NAN : target_local_ned.afx;
setpoint.acceleration[1] = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? (float)NAN : target_local_ned.afy;
setpoint.acceleration[2] = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? (float)NAN : target_local_ned.afz;
```

---

*记录时间：2026-06-01*
*版本：v1.1（新增Stall Detection完整代码）*
EOF
echo "核心代码片段已生成"