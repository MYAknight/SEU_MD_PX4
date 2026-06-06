#pragma once

// HUAQICCC Morphing Drone - Precomputed Motor Parameter Lookup Table
// Generated automatically from SDF kinematics
// Angle range: 0.0 to -0.50 rad, step: 0.05 rad

struct HuaqicccMotorParams {
    float px[4];  // motor 0=lb, 1=lf, 2=rb, 3=rf
    float py[4];
    float pz[4];
};

static constexpr int HUAQICCC_LUT_SIZE = 11;
static constexpr float HUAQICCC_LUT_STEP = 0.05f;
static constexpr float HUAQICCC_LUT_MAX_ANGLE = -0.5f;

// Lookup table: index 0 = angle 0.0, index i = angle -i*STEP
// 注意：每个结构体初始化用一层花括号，内部3个数组成员各用一层
static constexpr HuaqicccMotorParams huaqiccc_motor_lut[HUAQICCC_LUT_SIZE] = {
    {   // index=0, angle=0.00 rad
        { -0.18773f, +0.27137f, -0.19233f, +0.26727f },  // px
        { -0.24887f, -0.22907f, +0.24553f, +0.23303f },  // py
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }   // pz  <- 注意：最后一个成员初始化不要加逗号(C++11前兼容)
    },
    {   // index=1, angle=-0.05 rad
        { -0.19459f, +0.26493f, -0.19910f, +0.26055f },
        { -0.24442f, -0.24759f, +0.24094f, +0.25142f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=2, angle=-0.10 rad
        { -0.20096f, +0.25782f, -0.20538f, +0.25317f },
        { -0.23941f, -0.26554f, +0.23580f, +0.26924f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=3, angle=-0.15 rad
        { -0.20683f, +0.25007f, -0.21114f, +0.24517f },
        { -0.23386f, -0.28289f, +0.23013f, +0.28645f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=4, angle=-0.20 rad
        { -0.21218f, +0.24170f, -0.21637f, +0.23655f },
        { -0.22779f, -0.29959f, +0.22394f, +0.30300f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=5, angle=-0.25 rad
        { -0.21700f, +0.23273f, -0.22106f, +0.22735f },
        { -0.22120f, -0.31560f, +0.21725f, +0.31885f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=6, angle=-0.30 rad
        { -0.22127f, +0.22318f, -0.22519f, +0.21758f },
        { -0.21412f, -0.33088f, +0.21007f, +0.33395f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=7, angle=-0.35 rad
        { -0.22499f, +0.21307f, -0.22876f, +0.20726f },
        { -0.20656f, -0.34538f, +0.20243f, +0.34828f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=8, angle=-0.40 rad
        { -0.22814f, +0.20243f, -0.23175f, +0.19644f },
        { -0.19854f, -0.35909f, +0.19434f, +0.36180f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=9, angle=-0.45 rad
        { -0.23072f, +0.19129f, -0.23416f, +0.18512f },
        { -0.19009f, -0.37195f, +0.18582f, +0.37447f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    },
    {   // index=10, angle=-0.50 rad
        { -0.23272f, +0.17967f, -0.23599f, +0.17334f },
        { -0.18121f, -0.38394f, +0.17689f, +0.38626f },
        { -0.03842f, -0.03842f, -0.03842f, -0.03842f }
    }
};

// Inline helper: get motor params by arm angle (with nearest-neighbor lookup)
static inline void huaqiccc_get_motor_params(float arm_angle, float px[4], float py[4], float pz[4]) {
    // Clamp angle to valid range
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
