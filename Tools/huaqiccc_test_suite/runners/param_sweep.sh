#!/bin/bash
# Parameter sweep for perching admittance/compliance/adaptive schemes.
# Runs 13_grasp_16cm.sh repeatedly with different MPCA_PC_ADM_* env variables
# and aggregates hold/contact-stage statistics (plus monitor-stage pitch) from the resulting CSV logs.

set -e

RUNNER="$(dirname "$0")/13_grasp_16cm.sh"
LOG_DIR="$HOME/huaqiccc_logs"
OUT_DIR="$HOME/huaqiccc_sweep_results"
mkdir -p "$OUT_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY="$OUT_DIR/sweep_${TIMESTAMP}.csv"

echo "phase,label,run,act_z_mean,act_z_std,max_hold_pitch_deg,max_abs_monitor_pitch_deg,max_f_est,max_delta_p,success,csv_file" > "$SUMMARY"

# Helper: run one configuration N times
run_config() {
    local label=$1
    local env_vars=$2
    local n_runs=$3

    for i in $(seq 1 $n_runs); do
        echo "========================================"
        echo " [$label] run $i/$n_runs"
        echo " env: $env_vars"
        echo "========================================"

        # Clean up any stale simulation processes before starting
        pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
        sleep 2

        # Run test. env_vars string is split into separate args for env.
        env $(echo "$env_vars") "$RUNNER" 0 >/tmp/sweep_run.log 2>&1 || true

        # Find the most recent CSV
        latest=$(ls -t "$LOG_DIR"/grasp_test_success_*.csv 2>/dev/null | head -1)
        if [ -z "$latest" ]; then
            echo "[WARN] No CSV found for $label run $i"
            echo "$label,$label,$i,N/A,N/A,N/A,N/A,N/A,0," >> "$SUMMARY"
            continue
        fi

        # Extract hold/contact-stage + monitor-stage statistics
        # CSV columns: phase,time,sp_x,sp_y,sp_z,act_x,act_y,act_z,ax,ay,az,a_norm,efo_mag,contact_state,should_close,delta_p,f_est,pitch_deg,px4_phase
        awk -F',' -v label="$label" -v run="$i" -v csv="$latest" '
            $1 == "monitor" {
                z_sum += $8; z2_sum += $8*$8; n++
                p = $18; if (p < 0) p = -p; if (p > max_mp) max_mp = p
            }
            $1 == "hold" || $1 == "contact" {
                p = $18; if (p < 0) p = -p; if (p > max_hp) max_hp = p
                f = $17; if (f < 0) f = -f; if (f > max_f) max_f = f
                d = $16; if (d < 0) d = -d; if (d > max_d) max_d = d
            }
            END {
                if (n > 0) {
                    zm = z_sum / n
                    zvar = z2_sum/n - zm*zm; zstd = (zvar > 0) ? sqrt(zvar) : 0
                    printf "%s,%s,%d,%.4f,%.6f,%.2f,%.2f,%.3f,%.4f,1,%s\n", label, label, run, zm, zstd, max_hp, max_mp, max_f, max_d, csv
                } else {
                    printf "%s,%s,%d,N/A,N/A,N/A,N/A,N/A,N/A,0,%s\n", label, label, run, csv
                }
            }
        ' "$latest" >> "$SUMMARY"

        echo "[DONE] $label run $i -> $latest"
    done
}

# Baseline
run_config "Baseline" "" 3

# Scheme A: admittance, KA x FD
run_config "A_KA0.005_FD1.0" "MPCA_PC_ADM_KA=0.005 MPCA_PC_ADM_FD=1.0" 2
run_config "A_KA0.010_FD1.0" "MPCA_PC_ADM_KA=0.010 MPCA_PC_ADM_FD=1.0" 2
run_config "A_KA0.010_FD2.0" "MPCA_PC_ADM_KA=0.010 MPCA_PC_ADM_FD=2.0" 2

# Scheme B: position-error + velocity damping, KP x KV
run_config "B_KP0.3"       "MPCA_PC_ADM_KP=0.3 MPCA_PC_ADM_KV=0.0" 2
run_config "B_KP0.5"       "MPCA_PC_ADM_KP=0.5 MPCA_PC_ADM_KV=0.0" 2
run_config "B_KP0.5_KV0.2" "MPCA_PC_ADM_KP=0.5 MPCA_PC_ADM_KV=0.2" 2

# Scheme C: adaptive pitch+error, KC x (W1,W2)
run_config "C_KC0.005_W11" "MPCA_PC_ADM_KC=0.005 MPCA_PC_ADM_W1=1 MPCA_PC_ADM_W2=1" 2
run_config "C_KC0.010_W11" "MPCA_PC_ADM_KC=0.010 MPCA_PC_ADM_W1=1 MPCA_PC_ADM_W2=1" 2
run_config "C_KC0.010_W21" "MPCA_PC_ADM_KC=0.010 MPCA_PC_ADM_W1=2 MPCA_PC_ADM_W2=1" 2

echo "========================================"
echo "Parameter sweep complete."
echo "Summary: $SUMMARY"
cat "$SUMMARY"
