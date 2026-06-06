#!/bin/bash
# MPC parameter tuning script - runs multiple alpha/r_delta combinations

set -e

# Base parameters
export MPCA_MODE=3
export MPCA_FF_EN=1
export MPCA_FF_BLEND=0.3
export MPCA_FF_MASS=1.5

# Test matrix: (alpha, r_delta, label)
combinations=(
    "10.0 0.005 a10_rd5"
    "15.0 0.005 a15_rd5"
    "20.0 0.005 a20_rd5"
    "25.0 0.005 a25_rd5"
    "20.0 0.002 a20_rd2"
    "20.0 0.010 a20_rd10"
    "30.0 0.005 a30_rd5"
)

for combo in "${combinations[@]}"; do
    alpha=$(echo $combo | awk '{print $1}')
    rdelta=$(echo $combo | awk '{print $2}')
    label=$(echo $combo | awk '{print $3}')
    
    echo "========================================"
    echo "  Testing: alpha=$alpha, r_delta=$rdelta ($label)"
    echo "========================================"
    
    export MPCA_MPC_ALPHA=$alpha
    export MPCA_MPC_R_DELTA=$rdelta
    
    bash /home/a/huaqiccc_test_suite/runners/01_flatness_circle.sh 3 1 > /tmp/run_${label}.log 2>&1
    
    # Copy result with label
    latest=$(ls -t ~/huaqiccc_logs/huaqiccc_flatness_m3_ff1_with_algo_*.csv | head -1)
    if [ -n "$latest" ]; then
        cp "$latest" "~/huaqiccc_logs/huaqiccc_flatness_${label}.csv"
        echo "  Saved: huaqiccc_flatness_${label}.csv"
    fi
    
    sleep 5
done

echo "========================================"
echo "  All tests complete"
echo "========================================"
