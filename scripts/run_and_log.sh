#!/bin/bash
# Run OKVIS2-X and record total processing time + yaml parameters used.
#
# Usage:
#   ./run_and_log.sh <okvis_config.yaml> <se_config.yaml> <dataset_path> [output_dir]
#
# Example:
#   ./run_and_log.sh config/vbr/okvis2.yaml config/vbr/se2.yaml Dataset/pa_1 result/pa_1

set -e

# ── argument checks ──────────────────────────────────────────────────────────
if [ "$#" -lt 3 ]; then
    echo "Usage: $0 <okvis_config.yaml> <se_config.yaml> <dataset_path> [output_dir]"
    exit 1
fi

OKVIS_CFG="$1"
SE_CFG="$2"
DATASET="$3"
OUTPUT_DIR="${4:-result/default}"

EXECUTABLE="$(dirname "$0")/../build/okvis2x_app_synchronous"
if [ ! -f "$EXECUTABLE" ]; then
    echo "ERROR: executable not found at $EXECUTABLE"
    exit 1
fi

# ── prepare log directory ────────────────────────────────────────────────────
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="$OUTPUT_DIR/logs"
mkdir -p "$LOG_DIR"

RUN_LOG="$LOG_DIR/run_${TIMESTAMP}.log"
SUMMARY_FILE="$LOG_DIR/summary.csv"

# Write CSV header if file does not exist yet
if [ ! -f "$SUMMARY_FILE" ]; then
    echo "timestamp,okvis_config,se_config,dataset,output_dir,processing_time_s,processing_time_okvis_only_s,exit_code" > "$SUMMARY_FILE"
fi

# ── extract key yaml parameters for the log ──────────────────────────────────
extract_yaml_param() {
    local file="$1"
    local key="$2"
    grep -m1 "${key}:" "$file" 2>/dev/null | sed 's/.*: *//' | tr -d ',' | awk '{print $1}' || echo "N/A"
}

echo "============================================================" | tee -a "$RUN_LOG"
echo "  OKVIS2-X Run  |  $TIMESTAMP"                              | tee -a "$RUN_LOG"
echo "============================================================" | tee -a "$RUN_LOG"
echo ""                                                              | tee -a "$RUN_LOG"
echo "--- OKVIS config: $OKVIS_CFG ---"                            | tee -a "$RUN_LOG"

# Key frontend / estimator parameters worth tracking
for KEY in detection_threshold absolute_threshold matching_threshold \
           max_num_keypoints keyframe_overlap use_cnn \
           num_keyframes num_loop_closure_frames num_imu_frames \
           do_loop_closures do_final_ba enforce_realtime \
           realtime_time_limit realtime_num_threads \
           full_graph_iterations full_graph_num_threads \
           image_delay; do
    VAL=$(extract_yaml_param "$OKVIS_CFG" "$KEY")
    printf "  %-35s %s\n" "$KEY:" "$VAL" | tee -a "$RUN_LOG"
done

if [ -f "$SE_CFG" ]; then
    echo ""                                  | tee -a "$RUN_LOG"
    echo "--- SE config: $SE_CFG ---"       | tee -a "$RUN_LOG"
    for KEY in map_size_m voxel_dim T_MW_factor enable_meshing; do
        VAL=$(extract_yaml_param "$SE_CFG" "$KEY")
        printf "  %-35s %s\n" "$KEY:" "$VAL" | tee -a "$RUN_LOG"
    done
fi

echo ""                                                             | tee -a "$RUN_LOG"
echo "Dataset  : $DATASET"                                         | tee -a "$RUN_LOG"
echo "Output   : $OUTPUT_DIR"                                      | tee -a "$RUN_LOG"
echo "Log file : $RUN_LOG"                                         | tee -a "$RUN_LOG"
echo ""                                                             | tee -a "$RUN_LOG"
echo "--- Program output ---"                                       | tee -a "$RUN_LOG"

# ── run the program ──────────────────────────────────────────────────────────
EXIT_CODE=0
"$EXECUTABLE" "$OKVIS_CFG" "$SE_CFG" "$DATASET" "$OUTPUT_DIR" 2>&1 | tee -a "$RUN_LOG" || EXIT_CODE=${PIPESTATUS[0]}

# ── extract timing from log ──────────────────────────────────────────────────
# Matches lines like:
#   total processing time 123.456 s
#   total processing time OKVIS only 98.765 s
PROC_TIME=$(grep -oP "(?<=^.*total processing time )[\d.]+" "$RUN_LOG" | \
            grep -v "OKVIS only" | tail -1 || echo "N/A")
PROC_TIME_OKVIS=$(grep -oP "(?<=^.*total processing time OKVIS only )[\d.]+" "$RUN_LOG" | \
                  tail -1 || echo "N/A")

# Use less strict fallback if the above returns nothing (glog prefixes lines with I/W/E + timestamp)
if [ "$PROC_TIME" = "N/A" ] || [ -z "$PROC_TIME" ]; then
    PROC_TIME=$(grep "total processing time" "$RUN_LOG" | grep -v "OKVIS only" | \
                grep -oP "[\d]+\.[\d]+" | tail -1 || echo "N/A")
fi
if [ "$PROC_TIME_OKVIS" = "N/A" ] || [ -z "$PROC_TIME_OKVIS" ]; then
    PROC_TIME_OKVIS=$(grep "total processing time OKVIS only" "$RUN_LOG" | \
                      grep -oP "[\d]+\.[\d]+" | tail -1 || echo "N/A")
fi

# ── print summary ────────────────────────────────────────────────────────────
echo ""                                                                   | tee -a "$RUN_LOG"
echo "============================================================"       | tee -a "$RUN_LOG"
echo "  RESULTS"                                                           | tee -a "$RUN_LOG"
echo "============================================================"       | tee -a "$RUN_LOG"
echo "  Total processing time          : ${PROC_TIME} s"                  | tee -a "$RUN_LOG"
echo "  Total processing time (OKVIS)  : ${PROC_TIME_OKVIS} s"           | tee -a "$RUN_LOG"
echo "  Exit code                      : $EXIT_CODE"                      | tee -a "$RUN_LOG"
echo "============================================================"       | tee -a "$RUN_LOG"

# ── append one line to the shared CSV ────────────────────────────────────────
echo "${TIMESTAMP},${OKVIS_CFG},${SE_CFG},${DATASET},${OUTPUT_DIR},${PROC_TIME},${PROC_TIME_OKVIS},${EXIT_CODE}" \
    >> "$SUMMARY_FILE"

echo ""
echo "Summary appended to: $SUMMARY_FILE"

exit "$EXIT_CODE"
