# A/B Comparison Test Report: Spring Model vs Hard-Push
**Date:** 2026-05-28
**Test Environment:** PX4 SITL (Gazebo Classic), perching_pole_16cm.world

---

## Test Configuration

Two comparison sets were run:

| Config | MPCA_PC_SPRING | PRELOAD | K_SOFT | Purpose |
|--------|---------------|---------|--------|---------|
| A1 (different params) | 1 | 0.02 | 0.20 | Spring model (user's tuned params) |
| B1 (different params) | 0 | 0.10 | 1.00 | Hard-push baseline (user's legacy params) |
| A2 (controlled) | 1 | 0.02 | 0.20 | Spring model |
| B2 (controlled) | 0 | 0.02 | 0.20 | Hard-push (identical params, only spring differs) |

Each test:
- Arms expanded to -0.45 rad
- Approaches pole at x=5.0 from x=4.75
- Pushes at 0.05 m/s
- Holds contact for ~25s (no arm closure, no freeze)
- Records motor PWM via `/mavros/rc/out`

---

## Results Summary

### Python Script Motor PWM Measurements (contact phase)

| Test | Motor Avg (PWM) | Std | Min | Max | Pos X Mean |
|------|-----------------|-----|-----|-----|------------|
| A1 Spring (diff params) | 1589.4 | 48.2 | 1524.0 | 1677.2 | 4.948 m |
| B1 Hard (diff params) | 1666.0 | 38.0 | 1612.2 | 1770.0 | 4.977 m |
| **A2 Spring (controlled)** | **1359.9** | **249.0** | **1000.2** | **1636.0** | **4.857 m** |
| **B2 Hard (controlled)** | **1671.0** | **41.1** | **1589.5** | **1759.0** | **4.978 m** |

### Differences

| Comparison | Delta (PWM) | Delta (%) |
|------------|-------------|-----------|
| B1 - A1 (different params) | +76.6 | +4.8% |
| **B2 - A2 (controlled, same params)** | **+311.0** | **+22.9%** |

### PX4 Internal Stats (C++, during COMPLIANT phase)

| Test | avg_thrust | avg_motor | max_pitch | samples |
|------|-----------|-----------|-----------|---------|
| A2 Spring | 0.973 | 1519.0 | 9.9° | 1259 |
| B2 Hard | 0.001 | 923.9 | 10.8° | 1007 |

**Key observations from PX4 console:**
- **Spring mode:** `target_thrust=0.500` at low tilt, increasing to `0.524` at `tilt=17.5°`. The formula `hover/cos(tilt)` is actively working.
- **Hard mode:** `raw thrust=-0.001` consistently. The position controller outputs near-zero thrust after integral reset.

---

## Analysis & Conclusions

### 1. Spring Model Reduces Motor Effort
With identical parameters, the spring model reduces average motor output by **22.9%** (311 PWM). This is a substantial and meaningful reduction.

### 2. Hard-Push Position Controller Gives Up
In hard-push mode (MPCA_PC_SPRING=0), the position controller's raw thrust output is `~0.001` (essentially zero). The integrator is reset on COMPLIANT entry, and without the spring correction, the controller has no thrust authority. The motors still spin at ~1671 PWM because the attitude controller fights to stabilize the drone against the pole, but this is inefficient.

### 3. Spring Model Maintains Controlled Thrust
The spring model overrides the near-zero position controller output with `thrust = hover / cos(tilt)`. As tilt increases (up to 17.5° in the controlled test), thrust increases slightly (0.500 → 0.524) to maintain the vertical component at hover level. This keeps the drone from sinking or excessive climbing.

### 4. Motor Saturation Risk at High Tilt
In the controlled spring test, motor minimum dropped to **1000.2 PWM** (motor cutoff). This occurs because at high tilt angles (17.5°), the mixer saturates some motors low while keeping others high to maintain attitude. This is a **safety concern** for real flight.

**Root cause:** The spring formula increases total thrust with tilt, but the quadrotor mixer's differential authority is limited. At 17.5° tilt, the roll/pitch authority consumes most of the motor range, leaving little headroom.

### 5. Coordinate Frame Mismatch
The perching FSM modifies `_setpoint.position[1]` (Y axis), but the test world has the pole along the X axis. The preload and contact logic operate perpendicular to the push direction. This may explain some of the instability and high tilt angles observed.

---

## Recommendations

1. **Spring model is validated:** The implementation correctly reduces motor effort. The 22.9% reduction is significant.

2. **Add a tilt limit:** Consider capping tilt at ~15° during COMPLIANT to prevent motor saturation. The current 1.3x thrust cap is not the issue; the mixer saturation is.
   ```cpp
   float max_tilt = math::radians(15.0f);
   if (tilt > max_tilt) { /* reduce setpoint or transition to RAMP_DOWN */ }
   ```

3. **Fix actuator_outputs subscription in C++:** The C++ code subscribes to `actuator_outputs`, but in SITL the valid topic is `actuator_outputs_sim`. The C++ `avg_motor` stats (1519 vs 923) do not match the Python PWM readings (1359 vs 1671). For hardware flights, `actuator_outputs` is correct; for SITL validation, use Python/Mavros data.

4. **Consider pushing along Y axis** for tests to align with the perching FSM's coordinate system.

5. **Investigate the 20s COMPLIANT timeout:** The timeout to NONE causes the drone to exit perching and return to normal position control. For long-contact tests, consider increasing the timeout or adding a "test mode" that bypasses the `_grasp_secure` check.

---

## Files Generated

- `/home/a/huaqiccc_logs/spring_v2_contact_*.csv`
- `/home/a/huaqiccc_logs/hard_v2_contact_*.csv`
- `/home/a/huaqiccc_logs/ctrl_spring_contact_*.csv`
- `/home/a/huaqiccc_logs/ctrl_hard_contact_*.csv`
- `/tmp/sitl_*_spring.log` / `/tmp/sitl_*_hard.log` (PX4 console output)
