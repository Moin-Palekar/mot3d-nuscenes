#!/usr/bin/env bash
# Runs one tracking experiment end to end and prints its headline numbers.
#
# Usage: scripts/run_experiment.sh configs/experiments/<name>.json
# Output: results/experiments/<name>/ (submission, scorer output, log)
set -euo pipefail

config="$1"
name="$(basename "$config" .json)"
out="results/experiments/$name"
mkdir -p "$out"

./build/apps/run_tracker data/detections/infos_val_10sweeps_withvelo_filter_True.json \
  data/v1.0-trainval "$out/submission.json" "$config" > "$out/run.log"
scripts/evaluate.sh "$out/submission.json" "$out/eval" > "$out/eval.log" 2>&1

python3 - "$out/eval/metrics_summary.json" "$name" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
print(f"{sys.argv[2]:<16} AMOTA {m['amota']:.3f}  recall {m['recall']:.3f}  IDS {m['ids']}  TID {m['tid']:.2f}")
PY
