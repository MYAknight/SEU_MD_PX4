#!/bin/bash
# huaqiccc Test Suite Entry Point
# Usage: ./run_huaqiccc_test.sh <test_name> [args...]
#
# Available tests:
#   flatness          - Circle trajectory with flatness feedforward (MPC+FF)
#   simplified        - Simplified flight test (any mode)
#   aggressive        - Aggressive trajectory test
#   pole_collision    - Pole collision / perching test
#   pole_pass         - Pole pass verification test
#   grasp             - 16cm pole grasping test
#   batch_flatness    - Batch comparison: PID/GS-PID/LQR/MPC±FF
#   batch_aggressive  - Batch aggressive trajectory (2 repeats × 5 configs)
#   mpc_tuning        - MPC parameter sweep (alpha × r_delta matrix)
#
# Examples:
#   ./run_huaqiccc_test.sh flatness 3 1          # MPC+FF mode
#   ./run_huaqiccc_test.sh simplified 0          # Original PID mode
#   ./run_huaqiccc_test.sh batch_flatness        # Run all comparisons

set -e

SUITE_DIR="$HOME/huaqiccc_test_suite/runners"
TEST="${1:-flatness}"
shift || true

case "$TEST" in
    flatness|circle)
        bash "$SUITE_DIR/01_flatness_circle.sh" "$@"
        ;;
    simplified)
        bash "$SUITE_DIR/02_simplified_flight.sh" "$@"
        ;;
    aggressive)
        bash "$SUITE_DIR/03_aggressive_trajectory.sh" "$@"
        ;;
    pole_collision|perching)
        bash "$SUITE_DIR/11_pole_collision.sh" "$@"
        ;;
    pole_pass)
        bash "$SUITE_DIR/12_pole_pass_verify.sh" "$@"
        ;;
    grasp|grasping)
        bash "$SUITE_DIR/13_grasp_16cm.sh" "$@"
        ;;
    batch_flatness|batch)
        bash "$SUITE_DIR/21_batch_flatness_comparison.sh"
        ;;
    batch_aggressive)
        bash "$SUITE_DIR/22_batch_aggressive_repeated.sh"
        ;;
    mpc_tuning|tuning)
        bash "$SUITE_DIR/23_mpc_parameter_sweep.sh"
        ;;
    *)
        echo "Unknown test: $TEST"
        echo "Available: flatness, simplified, aggressive, pole_collision, pole_pass, grasp,"
        echo "           batch_flatness, batch_aggressive, mpc_tuning"
        exit 1
        ;;
esac
