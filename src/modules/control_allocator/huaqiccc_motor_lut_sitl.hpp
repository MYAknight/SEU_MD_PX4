#pragma once

// HUAQICCC Morphing Drone - SITL-specific Motor Parameter Lookup Table
// Generated from Gazebo SDF geometry (huaqiccc.sdf) 2026-06-12.
// Use this LUT in SITL builds; real hardware should use huaqiccc_motor_lut.hpp.
// Coordinate convention: PX4 FRD (X=Front, Y=Right, Z=Down).
// Order: 0=rb, 1=rf, 2=lb, 3=lf  (matches ControlAllocator.cpp column order)
// Angle range: 0.0 to -0.40 rad (mechanical limit), step: 0.05 rad

#include "huaqiccc_motor_lut.hpp"  // for HuaqicccMotorParams, helper signature

static constexpr HuaqicccMotorParams huaqiccc_motor_lut_sitl[HUAQICCC_LUT_SIZE] = {
    {   // index=0, angle=0.00 rad
        { -0.20002, +0.25965, -0.19535, +0.26371 },  // px
        { +0.24546, +0.23303, -0.24893, -0.22916 },  // py
        { -0.03945, -0.03945, -0.03945, -0.03945 }   // pz
    },
    {   // index=1, angle=-0.05 rad
        { -0.20661, +0.25311, -0.20202, +0.25745 },
        { +0.24087, +0.25142, -0.24448, -0.24767 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=2, angle=-0.10 rad
        { -0.21268, +0.24593, -0.20820, +0.25055 },
        { +0.23573, +0.26925, -0.23947, -0.26562 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=3, angle=-0.15 rad
        { -0.21822, +0.23814, -0.21385, +0.24302 },
        { +0.23006, +0.28646, -0.23392, -0.28297 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=4, angle=-0.20 rad
        { -0.22322, +0.22976, -0.21896, +0.23488 },
        { +0.22388, +0.30301, -0.22784, -0.29966 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=5, angle=-0.25 rad
        { -0.22765, +0.22080, -0.22353, +0.22616 },
        { +0.21719, +0.31886, -0.22126, -0.31567 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=6, angle=-0.30 rad
        { -0.23152, +0.21130, -0.22753, +0.21687 },
        { +0.21002, +0.33398, -0.21417, -0.33094 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=7, angle=-0.35 rad
        { -0.23480, +0.20126, -0.23097, +0.20704 },
        { +0.20237, +0.34831, -0.20661, -0.34544 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=8, angle=-0.40 rad
        { -0.23750, +0.19073, -0.23383, +0.19670 },
        { +0.19428, +0.36183, -0.19859, -0.35914 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=9, angle=-0.45 rad
        { -0.23960, +0.17972, -0.23610, +0.18586 },
        { +0.18577, +0.37451, -0.19013, -0.37200 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    },
    {   // index=10, angle=-0.50 rad
        { -0.24110, +0.16826, -0.23778, +0.17456 },
        { +0.17684, +0.38630, -0.18125, -0.38398 },
        { -0.03945, -0.03945, -0.03945, -0.03945 }
    }
};

// SITL helper: get motor params by arm angle (with nearest-neighbor lookup)
static inline void huaqiccc_get_motor_params_sitl(float arm_angle, float px[4], float py[4], float pz[4]) {
    if (arm_angle > 0.0f) { arm_angle = 0.0f; }
    if (arm_angle < HUAQICCC_LUT_MAX_ANGLE) { arm_angle = HUAQICCC_LUT_MAX_ANGLE; }
    int idx = static_cast<int>(roundf(fabsf(arm_angle) / HUAQICCC_LUT_STEP));
    if (idx < 0) { idx = 0; }
    if (idx >= HUAQICCC_LUT_SIZE) { idx = HUAQICCC_LUT_SIZE - 1; }
    for (int i = 0; i < 4; ++i) {
        px[i] = huaqiccc_motor_lut_sitl[idx].px[i];
        py[i] = huaqiccc_motor_lut_sitl[idx].py[i];
        pz[i] = huaqiccc_motor_lut_sitl[idx].pz[i];
    }
}
