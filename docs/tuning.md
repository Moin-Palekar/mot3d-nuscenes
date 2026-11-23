# Tuning log

Every run: CenterPoint val detections (150 scenes, 6,019 frames), scored with the official
nuScenes tracking evaluation. Each experiment changes one setting from the defaults; its config
is in `configs/experiments/`. Reproduce with `scripts/run_experiment.sh <config>`.

| Experiment | Change | AMOTA | Recall | ID switches | TID (s) |
|---|---|---|---|---|---|
| baseline | defaults | 0.588 | 0.657 | 1367 | 1.06 |
| max_cost_5 | gate 3.64 -> 5.0 | 0.571 | 0.632 | 1245 | 1.15 |
| max_misses_3 | max_misses 2 -> 3 | 0.580 | 0.640 | 1038 | 1.04 |
| min_hits_1 | min_hits 2 -> 1 | **0.591** | 0.643 | 1194 | 0.82 |

Sanity gate: ground-truth boxes fed in as detections score 0.917 AMOTA (91 ID switches), so the
tracker logic is sound and the remaining gap comes from detection noise and modelling choices.

Finding: these lifecycle and gating settings move AMOTA by about +/-0.02. Larger gains need
changes to how the tracker uses the detections (next: detector velocity as a measurement).
