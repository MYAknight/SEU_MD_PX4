# Huaqiccc 控制参数基线记录
# 生成时间: 2026-05-23
# 用于参数调优恢复

## 1. GS-PID LUT (AdvancedPositionControl.cpp line 45-57)
static constexpr GainSet default_lut[GS_LUT_SIZE] = {
    /* idx=0,  angle=0.00  */ {4.00f, 1.00f, 2.20f, 0.40f, 0.20f, 4.00f, 2.00f, 0.0f},
    /* idx=1,  angle=-0.05 */ {4.08f, 1.02f, 2.24f, 0.41f, 0.20f, 4.08f, 2.04f, 0.0f},
    /* idx=2,  angle=-0.10 */ {4.16f, 1.04f, 2.28f, 0.42f, 0.21f, 4.16f, 2.08f, 0.0f},
    /* idx=3,  angle=-0.15 */ {4.24f, 1.06f, 2.32f, 0.43f, 0.21f, 4.24f, 2.12f, 0.0f},
    /* idx=4,  angle=-0.20 */ {4.32f, 1.08f, 2.36f, 0.44f, 0.22f, 4.32f, 2.16f, 0.0f},
    /* idx=5,  angle=-0.25 */ {4.40f, 1.10f, 2.40f, 0.45f, 0.22f, 4.40f, 2.20f, 0.0f},
    /* idx=6,  angle=-0.30 */ {4.48f, 1.12f, 2.44f, 0.46f, 0.23f, 4.48f, 2.24f, 0.0f},
    /* idx=7,  angle=-0.35 */ {4.56f, 1.14f, 2.48f, 0.47f, 0.23f, 4.56f, 2.28f, 0.0f},
    /* idx=8,  angle=-0.40 */ {4.64f, 1.16f, 2.52f, 0.48f, 0.24f, 4.64f, 2.32f, 0.0f},
    /* idx=9,  angle=-0.45 */ {4.72f, 1.18f, 2.56f, 0.49f, 0.24f, 4.72f, 2.36f, 0.0f},
    /* idx=10, angle=-0.50 */ {4.80f, 1.20f, 2.60f, 0.50f, 0.25f, 4.80f, 2.40f, 0.0f}
};

## 2. LQR LUT (AdvancedPositionControl.cpp line 65-77)
static constexpr GainSet lqr_lut[GS_LUT_SIZE] = {
    /* idx=0,  angle=0.00  */ {4.000f, 1.000f, 2.915f, 0.400f, 0.200f, 2.000f, 2.000f, 0.0f},
    /* idx=1,  angle=-0.05 */ {4.087f, 1.025f, 2.953f, 0.410f, 0.205f, 2.037f, 2.040f, 0.0f},
    /* idx=2,  angle=-0.10 */ {4.171f, 1.049f, 2.990f, 0.420f, 0.210f, 2.073f, 2.080f, 0.0f},
    /* idx=3,  angle=-0.15 */ {4.254f, 1.072f, 3.026f, 0.430f, 0.215f, 2.108f, 2.120f, 0.0f},
    /* idx=4,  angle=-0.20 */ {4.336f, 1.095f, 3.061f, 0.440f, 0.220f, 2.143f, 2.160f, 0.0f},
    /* idx=5,  angle=-0.25 */ {4.416f, 1.118f, 3.095f, 0.450f, 0.225f, 2.176f, 2.200f, 0.0f},
    /* idx=6,  angle=-0.30 */ {4.494f, 1.140f, 3.129f, 0.460f, 0.230f, 2.209f, 2.240f, 0.0f},
    /* idx=7,  angle=-0.35 */ {4.572f, 1.162f, 3.161f, 0.470f, 0.235f, 2.241f, 2.280f, 0.0f},
    /* idx=8,  angle=-0.40 */ {4.648f, 1.183f, 3.193f, 0.480f, 0.240f, 2.273f, 2.320f, 0.0f},
    /* idx=9,  angle=-0.45 */ {4.722f, 1.204f, 3.224f, 0.490f, 0.245f, 2.304f, 2.360f, 0.0f},
    /* idx=10, angle=-0.50 */ {4.796f, 1.225f, 3.254f, 0.500f, 0.250f, 2.334f, 2.400f, 0.0f}
};

## 3. MPC Parameters (AdvancedPositionControl.hpp / .cpp)
- MPC_N = 5
- MPC_MAX_ITER = 20
- MPC_TOL = 1e-3f
- _mpc_alpha = 20.0f
- _mpc_u_min_xy = -3.0f
- _mpc_u_max_xy = 3.0f
- _mpc_u_min_z = -8.0f
- _mpc_u_max_z = 8.0f

### H Matrix:
{0.020375f, 0.0079f,   0.005525f, 0.00335f,  0.001475f}
{0.0079f,   0.0171f,   0.005075f, 0.00315f,  0.001425f}
{0.005525f, 0.005075f, 0.014625f, 0.00295f,  0.001375f}
{0.00335f,  0.00315f,  0.00295f,  0.01275f,  0.001325f}
{0.001475f, 0.001425f, 0.001375f, 0.001325f, 0.011275f}

### M Matrix:
{-0.5f,   -0.22f}
{-0.32f,  -0.166f}
{-0.18f,  -0.115f}
{-0.08f,  -0.069f}
{-0.02f,  -0.03f}

### g_const:
{0.095f, 0.066f, 0.04f, 0.019f, 0.005f}

## 4. PX4 Original Position Control Parameters (mc_pos_control_params.c)
- MPC_THR_MIN = 0.12f
- MPC_THR_HOVER = 0.5f
- MPC_USE_HTE = 1
- MPC_THR_XY_MARG = 0.3f
- MPC_THR_MAX = 1.0f
- MPC_Z_P = 1.0f
- MPC_Z_VEL_P_ACC = 4.0f
- MPC_Z_VEL_I_ACC = 2.0f
- MPC_Z_VEL_D_ACC = 0.0f
- MPC_XY_P = 0.95f
- MPC_XY_VEL_P_ACC = 1.8f
- MPC_XY_VEL_I_ACC = 0.4f
- MPC_XY_VEL_D_ACC = 0.2f
- MPC_TILTMAX_AIR = 45.0f
- MPC_ACC_HOR_MAX = 5.0f
- MPC_ACC_UP_MAX = 4.0f
- MPC_ACC_DOWN_MAX = 3.0f
- MPC_JERK_MAX = 8.0f
- MPC_JERK_AUTO = 4.0f

## 5. SITL Default Parameters (px4-rc.params)
- MPCA_MODE = 3 ( varies by test )
- MPC_XY_P = 1.5 ( set by Python test script )
