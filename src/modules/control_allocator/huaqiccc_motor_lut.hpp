#pragma once

// HUAQICCC Morphing Drone - Motor Parameter Lookup Table
// UPDATED 2026-06-11: matched to real hardware measurements
// Order: 0=rb, 1=rf, 2=lb, 3=lf  (matches ControlAllocator.cpp column order)
// Angle range: 0.0 (closed) to -0.40 rad (fully open, mechanical limit), step: 0.05 rad

struct HuaqicccMotorParams {
    float px[4];  // motor 0=rb, 1=rf, 2=lb, 3=lf
    float py[4];
    float pz[4];
};

static constexpr int HUAQICCC_LUT_SIZE = 11;
static constexpr float HUAQICCC_LUT_STEP = 0.05f;
static constexpr float HUAQICCC_LUT_MAX_ANGLE = -0.40f;  // mechanical limit

static constexpr HuaqicccMotorParams huaqiccc_motor_lut[HUAQICCC_LUT_SIZE] = {
    {   // index=0, angle=0.00 rad
        { -0.15500, +0.20500, -0.15500, +0.20500 },  // px
        { +0.21500, +0.16500, -0.21500, -0.16500 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=1, angle=-0.05 rad
        { -0.16177, +0.19828, -0.16186, +0.19856 },  // px
        { +0.21041, +0.18339, -0.21055, -0.18352 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=2, angle=-0.10 rad
        { -0.16805, +0.19090, -0.16823, +0.19145 },  // px
        { +0.20527, +0.20121, -0.20554, -0.20147 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=3, angle=-0.15 rad
        { -0.17381, +0.18290, -0.17410, +0.18370 },  // px
        { +0.19960, +0.21842, -0.19999, -0.21882 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=4, angle=-0.20 rad
        { -0.17904, +0.17428, -0.17945, +0.17533 },  // px
        { +0.19341, +0.23497, -0.19392, -0.23552 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=5, angle=-0.25 rad
        { -0.18373, +0.16508, -0.18427, +0.16636 },  // px
        { +0.18672, +0.25082, -0.18733, -0.25153 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=6, angle=-0.30 rad
        { -0.18786, +0.15531, -0.18854, +0.15681 },  // px
        { +0.17954, +0.26592, -0.18025, -0.26681 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=7, angle=-0.35 rad
        { -0.19143, +0.14499, -0.19226, +0.14670 },  // px
        { +0.17190, +0.28025, -0.17269, -0.28131 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=8, angle=-0.40 rad
        { -0.19442, +0.13417, -0.19541, +0.13606 },  // px
        { +0.16381, +0.29377, -0.16467, -0.29502 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=9, angle=-0.45 rad
        { -0.19683, +0.12285, -0.19799, +0.12492 },  // px
        { +0.15529, +0.30644, -0.15622, -0.30788 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    },
    {   // index=10, angle=-0.50 rad
        { -0.19866, +0.11107, -0.19999, +0.11330 },  // px
        { +0.14636, +0.31823, -0.14734, -0.31987 },  // py
        { +0.00000, +0.00000, +0.00000, +0.00000 }   // pz
    }
};

// Inline helper: get motor params by arm angle (with nearest-neighbor lookup)
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
