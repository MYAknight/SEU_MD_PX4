#!/bin/bash
set -e

# Clean up first
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 3

for mode in 0 3; do
    for ff in 0 1; do
        if [ "$mode" -eq 0 ] && [ "$ff" -eq 1 ]; then
            continue  # skip baseline with FF
        fi
        echo "========================================"
        echo "  Running test: MPCA_MODE=$mode, MPCA_FF_EN=$ff"
        echo "========================================"
        bash /home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/runners/01_flatness_circle.sh "$mode" "$ff" > "/tmp/run_test_m${mode}_ff${ff}.log" 2>&1
        # Wait for test completion
        while pgrep -f "huaqiccc_flatness_test" > /dev/null; do
            sleep 2
        done
        sleep 3
        CSV=$(ls -t /home/a/huaqiccc_logs/huaqiccc_flatness_m${mode}_ff${ff}_*.csv 2>/dev/null | head -1)
        if [ -n "$CSV" ]; then
            echo "CSV: $CSV"
            python3 /home/a/huaqiccc_logs/evaluate_flight.py "$CSV"
        fi
        echo ""
    done
done

echo "All tests completed!"
