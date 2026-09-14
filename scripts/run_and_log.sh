#!/usr/bin/env bash
# Run the DGVI-SLAM dataset application and retain its configuration and log.
set -euo pipefail

if (( $# < 2 )); then
  echo "Usage: $0 CONFIG DATASET [OUTPUT_DIR] [DEM.tif ...]" >&2
  exit 2
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
executable="${DGVI_SLAM_EXECUTABLE:-${repo_dir}/build/dgvi_slam_app}"
config="$1"
dataset="$2"
shift 2
output_dir="${1:-${repo_dir}/results/$(date -u +%Y%m%dT%H%M%SZ)}"
if (( $# > 0 )); then shift; fi

[[ -x "$executable" ]] || { echo "Executable not found: $executable" >&2; exit 2; }
[[ -f "$config" ]] || { echo "Configuration not found: $config" >&2; exit 2; }
[[ -d "$dataset" ]] || { echo "Dataset not found: $dataset" >&2; exit 2; }

# Each invocation gets its own directory to avoid overwriting earlier runs.
mkdir -p -- "$output_dir"
run_dir="$(mktemp -d "${output_dir}/run-XXXXXX")"
cp -- "$config" "${run_dir}/config.yaml"
printf '%q ' "$executable" "$config" "$dataset" "$run_dir" "$@" > "${run_dir}/command.txt"
printf '\n' >> "${run_dir}/command.txt"

set +e
"$executable" "$config" "$dataset" "$run_dir" "$@" 2>&1 | tee "${run_dir}/run.log"
pipeline_status=("${PIPESTATUS[@]}")
set -e
exit_code="${pipeline_status[0]}"
printf '%s\n' "$exit_code" > "${run_dir}/exit_code.txt"
echo "Run saved to: $run_dir"
if (( exit_code != 0 )); then exit "$exit_code"; fi
exit "${pipeline_status[1]}"
