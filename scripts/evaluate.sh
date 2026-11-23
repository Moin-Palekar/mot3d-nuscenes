#!/usr/bin/env bash
# Scores a tracking submission with the official nuScenes devkit on the val split.
#
# Usage: scripts/evaluate.sh [submission.json] [output_dir]
# Defaults: results/submission.json, results/eval
set -euo pipefail

submission="${1:-results/submission.json}"
output_dir="${2:-results/eval}"

python3 -m nuscenes.eval.tracking.evaluate "$submission" \
  --output_dir "$output_dir" \
  --eval_set val \
  --dataroot data \
  --version v1.0-trainval \
  --render_curves 0