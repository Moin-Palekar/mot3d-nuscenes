# Tuning log

Every run: CenterPoint val detections (150 scenes, 6,019 frames), scored with the official
nuScenes tracking evaluation. Each experiment changes one setting from the defaults; its config
is in `configs/experiments/`. Reproduce with `scripts/run_experiment.sh <config>`.

| Experiment | Change | AMOTA | Recall | ID switches | TID (s) |
|---|---|---|---|---|---|
| baseline | defaults | 0.588 | 0.657 | 1367 | 1.06 |
| max_cost_5 | gate 3.64 -> 5.0 | 0.571 | 0.632 | 1245 | 1.15 |
| max_misses_3 | max_misses 2 -> 3 | 0.580 | 0.640 | 1038 | 1.04 |
| min_hits_1 | min_hits 2 -> 1 | 0.591 | 0.643 | 1194 | 0.82 |
| velocity | detector speed blended into every update | 0.592 | 0.642 | 979 | 1.04 |
| velocity_min_hits_1 | velocity + min_hits 1 | 0.596 | 0.659 | 1104 | 0.74 |
| best_speed_std_0.5 | best + trust detector speed more (speed_std 0.5) | 0.587 | 0.661 | 1233 | 0.79 |
| best_speed_std_2 | best + trust detector speed less (speed_std 2.0) | 0.593 | 0.651 | 1184 | 0.76 |
| best_coast_0.5 | best + coasting, score x0.5 per missed frame | 0.593 | 0.664 | 660 | 0.77 |
| best_coast_0.25 | best + coasting, score x0.25 per missed frame | **0.598** | 0.677 | 602 | 0.75 |

Sanity gate: ground-truth boxes fed in as detections score 0.917 AMOTA (91 ID switches), so the
tracker logic is sound and the remaining gap comes from detection noise and modelling choices.

Finding: these lifecycle and gating settings move AMOTA by about +/-0.02. Larger gains need
changes to how the tracker uses the detections (next: detector velocity as a measurement).

Velocity: feeding the detector speed into each update cuts ID switches by 28% (1367 -> 979).
Combined with min_hits 1 the gains stack: 0.596 AMOTA, the best so far.

Coasting: reporting missed tracks at their predicted position, with the score lowered for each
missed frame, keeps identities through short gaps. With decay 0.25: 0.598 AMOTA, recall 0.677,
and 602 ID switches, 56% fewer than the untuned baseline. speed_std 1.0 beats both 0.5 and 2.0.
