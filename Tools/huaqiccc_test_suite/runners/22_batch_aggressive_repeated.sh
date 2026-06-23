#!/bin/bash
# Batch aggressive trajectory test - 2 repeats per config

set -e

export TRAJ_RADIUS=1.5
export TRAJ_PERIOD=15.0
export MPCA_FF_BLEND=0.3
export MPCA_FF_MASS=1.5
export MPCA_MPC_ALPHA=20.0
export MPCA_MPC_R_DELTA=0.005

# Configs: (mode, ff_en, label)
configs=(
    "3 1 mpc_ff"
    "3 0 mpc_noff"
    "2 0 lqr"
    "1 0 basic"
    "0 0 orig"
)

for repeat in 1 2; do
    for cfg in "${configs[@]}"; do
        mode=$(echo $cfg | awk '{print $1}')
        ffen=$(echo $cfg | awk '{print $2}')
        label=$(echo $cfg | awk '{print $3}')
        
        run_label="${label}_r${repeat}"
        echo "========================================"
        echo "  RUN: $run_label (mode=$mode, ff=$ffen)"
        echo "  Traj: R=$TRAJ_RADIUS, T=$TRAJ_PERIOD"
        echo "========================================"
        
        export MPCA_MODE=$mode
        export MPCA_FF_EN=$ffen
        
        bash /home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/runners/03_aggressive_trajectory.sh $mode $ffen > /tmp/run_${run_label}.log 2>&1
        
        # Rename result for clarity
        latest=$(ls -t ~/huaqiccc_logs/huaqiccc_flatness_m${mode}_ff${ffen}_*.csv | head -1)
        if [ -n "$latest" ]; then
            newname="${latest%/*}/huaqiccc_aggressive_${run_label}.csv"
            cp "$latest" "$newname"
            echo "  Saved: $(basename $newname)"
        fi
        
        sleep 3
    done
done

echo "========================================"
echo "  ALL TESTS COMPLETE"
echo "========================================"
