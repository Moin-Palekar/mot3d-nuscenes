# mot3d-nuscenes — System Specification

A tracking-by-detection 3D multi-object tracker for the nuScenes dataset, written in
C++17, evaluated with the official nuScenes tracking metrics, profiled, and wrapped in
ROS 2.

This document is the contract for the project. It defines what is being built, why,
what is explicitly excluded, and what "done" means at each milestone. It lives in the
repo at `docs/SPEC.md` and is committed before any tracker code is written.

---

## 1. What this is

Given a stream of 3D bounding-box detections produced by a **published, pre-trained
detector**, maintain consistent identities for objects across time and emit a nuScenes
tracking submission file that scores against the official metrics.

The system is a classical estimation pipeline, not a learned one:

```
detections (JSON)  ──▶  predict (EKF/CTRV)  ──▶  gate + cost matrix
                                                       │
                             lifecycle FSM  ◀── Hungarian assignment
                                    │
                                    ▼
                    confirmed tracks ──▶ submission JSON ──▶ nuscenes-devkit eval
                                     └──▶ ROS 2 MarkerArray ──▶ rviz2
```

### The single most important scope decision

**We do not train a detector.** Detections come from a released result file
(CenterPoint or MEGVII on nuScenes val). This removes GPU training, dataset
engineering, and weeks of hyperparameter tuning from the critical path.

It is also the *standard* experimental setup in the 3D MOT literature — AB3DMOT,
SimpleTrack, CBMOT and others all report numbers on top of published detections
precisely so that the tracker is isolated as the variable. That means your AMOTA is
directly comparable to published numbers using the same detection file. Third-party
validation for free.

---

## 2. Why this project

### 2.1 Against the two specific postings

| What it proves | Who needs it |
|---|---|
| Real C++17, not C++ as a checkbox | Boost ("exceptional skills in C++") |
| Kalman filtering, motion models, state estimation | Boost (localization / state estimation) |
| Calibration, sensor→ego→global transforms, point clouds | Percept (multi-view geometry) |
| Measured against a published baseline | Both — you currently have zero third-party validation |
| Latency profiling and optimization | Boost (edge-optimized, low-latency) |
| ROS 2, Docker, CI, unit tests | Boost (production code, testing, DevOps) |
| Public repo with a real README | Percept (open-source work) |

### 2.2 Against entry-level AV hiring generally

Every AV perception job description converges on roughly the same six asks. This
project hits five of them and the NAVSIM project hits the sixth.

1. **C++ in a real build system, not a LeetCode file.** CMake, targets, dependencies,
   tests, CI. Extremely common filter, rarely demonstrated by new grads.
2. **State estimation.** Kalman/EKF, motion models, covariance, gating. This is the
   single most reliably-asked interview topic in AV perception.
3. **Coordinate frames and calibration.** Sensor → ego → global, quaternions, SE(3)
   composition. The most common source of silent bugs in real stacks, and the thing
   interviewers probe to see whether you've actually touched a vehicle stack.
4. **Benchmark discipline.** Reproducing a published number, then ablating. Signals
   that you can be trusted with a metric.
5. **Latency awareness.** Profile, find the hot stage, optimize, report before/after.
6. **Learned perception at scale.** ← *not* this project. This is NAVSIM's job.

The two projects are deliberately complementary: this one is the classical /
production / C++ half, NAVSIM is the learned / research / Python half. Together they
cover the realistic surface area of an entry-level AV perception application.

### 2.3 Resume holes it closes

- No C++ artifact anywhere on the page.
- No ROS 2.
- No externally-validated number.
- No profiling or optimization evidence.

---

## 3. Explicitly out of scope

Each of these turns a four-week project into a four-month one. Resist all of them.

- **No detector training.** Not even fine-tuning.
- **No learned association**, re-ID embeddings, or appearance features.
- **No camera-image fusion** beyond consuming detections and drawing pictures.
- **No SLAM.** Ego pose comes from the dataset (`ego_pose` table).
- **No map priors**, no lane-graph-aware motion models.
- **No test-set submission.** Val only. The test server has a submission budget and
  nothing about it improves the resume line.
- **No multi-hypothesis tracking, no JPDA, no IMM.** Single-hypothesis, one motion
  model per configuration. IMM is a tempting stretch goal; it is a trap at this scope.

If a change requires touching any of the above, it goes in `docs/FUTURE_WORK.md`
instead of the code.

---

## 4. Data

### 4.1 nuScenes structure

- 1000 scenes × 20 s. Train 700 / val 150 / test 150.
- Keyframes ("samples") are annotated at **2 Hz** → 40 samples per scene, ~6k samples
  in val.
- **`dt = 0.5 s`** between tracker updates. This is a *large* timestep, which is why
  the motion model choice (CV vs CTRV) actually changes the result. On a 10 Hz stack
  it would barely matter.
- Tracking uses **7 classes**: `bicycle, bus, car, motorcycle, pedestrian, trailer,
  truck`. Detection has 10; `barrier`, `traffic_cone`, and `construction_vehicle` are
  dropped for tracking. Your loader must filter these out or the eval will complain.

### 4.2 What you actually have to download

| Artifact | Size | Needed for |
|---|---|---|
| `v1.0-mini` (full, with sensor data) | ~4 GB | M0–M2 development |
| `v1.0-trainval_meta` (metadata only) | ~500 MB | **M3–M5 scoring — this is all the eval needs** |
| Detection result JSON (val) — **obtained**, see §4.3 | 345 MB | M1 onward |
| `v1.0-trainval` sensor blobs (10 × ~30 GB) | ~300 GB | M6/M7 visuals only — download 1–2 blobs, not all 10 |

The eval script reads annotations from metadata. You can compute AMOTA on all 150 val
scenes without downloading a single LiDAR sweep. Plan the disk around that.

### 4.3 Detection input format

**Confirmed source.** `infos_val_10sweeps_withvelo_filter_True.json`, 345 MB, ASCII
JSON, downloaded from the CenterPoint model zoo (`configs/nusc/README.md`), hosted on
MIT OneDrive. Detector: `nusc_centerpoint_voxelnet_0075voxel_fix_bn_z` — 59.6 mAP /
66.8 NDS on val.

The whole file is a **single line** with no line terminators, so `head`, `less` and
most text tools are useless on it and editors will hang. Inspect with `jq`.

Top-level structure, verified from the first bytes:

```json
{"results": {"<sample_token>": [ {box}, {box}, ... ], ... }}
```

Each box, in nuScenes submission format:

```json
{
  "sample_token": "...",
  "translation": [x, y, z],          // GLOBAL frame, metres
  "size": [w, l, h],                 // note the ordering
  "rotation": [qw, qx, qy, qz],      // GLOBAL frame quaternion
  "velocity": [vx, vy],              // GLOBAL frame, m/s (may be NaN)
  "detection_name": "car",
  "detection_score": 0.87,
  "attribute_name": "vehicle.moving"
}
```

Three landmines, all of which will be unit-tested at M1:

- `size` is `[width, length, height]`, **not** `[l, w, h]`. Getting this wrong silently
  wrecks 3D IoU and produces a mediocre-but-plausible AMOTA that you'll chase for days.
- `velocity` can be `NaN`. Must be handled, not propagated into the filter.
- Boxes are in the **global frame already**. No transform needed to ingest them.

### 4.4 Coordinate frames (needed for M6/M7, and for the interview answer)

nuScenes defines three frames and two transforms:

```
sensor ──(calibrated_sensor: T_ego←sensor)──▶ ego ──(ego_pose: T_global←ego)──▶ global
```

- `calibrated_sensor`: static per (sensor, scene). Translation + quaternion.
- `ego_pose`: per timestamp, from the localization stack.
- Composition: `p_global = T_global←ego · T_ego←sensor · p_sensor`

We implement this chain properly — SE(3) as a first-class type, quaternion → rotation
matrix, inverse that is a transpose-and-negate rather than a general inverse — and
unit-test that a known point round-trips `sensor → global → sensor` to within 1e-9.
This is what Percept-type roles are checking for, and it's genuinely easy to get
subtly wrong.

---

## 5. Algorithm specification

### 5.1 Track state

Filtered state, **6-dimensional CTRV**:

```
x = [ px, py, pz, v, yaw, yaw_rate ]ᵀ        in the global frame
```

Box dimensions `(l, w, h)` are **not** in the filter. They are maintained per track as
an exponential moving average of the associated detections:

```
dims ← α · dims + (1 − α) · dims_meas,    α ≈ 0.9
```

Rationale: dimensions are nearly constant per object and putting them in the state adds
three dimensions of covariance for no accuracy gain, while making the filter slower and
harder to tune. Say exactly this if asked in an interview.

`pz` is modelled as a random walk (no vertical velocity).

### 5.2 Motion model — CTRV

Constant Turn Rate and Velocity. For `dt = 0.5 s`:

When `|yaw_rate| > ε`:
```
px' = px + (v / yaw_rate) · ( sin(yaw + yaw_rate·dt) − sin(yaw) )
py' = py + (v / yaw_rate) · ( −cos(yaw + yaw_rate·dt) + cos(yaw) )
yaw' = yaw + yaw_rate · dt
v' = v ,  yaw_rate' = yaw_rate
```

When `|yaw_rate| ≤ ε` (straight-line degenerate case):
```
px' = px + v · cos(yaw) · dt
py' = py + v · sin(yaw) · dt
yaw' = yaw
```

**The `ε` branch is mandatory.** Division by a near-zero yaw rate is the single most
common bug in CTRV implementations and it produces NaN state that propagates silently
into the covariance. Unit-tested explicitly, and it's a good interview story.

The Jacobian `F = ∂f/∂x` is derived analytically and hard-coded, not
finite-differenced. A unit test compares `F` against a central-difference
approximation to within 1e-6 — this catches derivation errors immediately and is a
cheap, professional-looking test.

**Process noise `Q`** is per-class, built from linear-acceleration and yaw-acceleration
noise (`σ_a`, `σ_yaw_ddot`). Pedestrians get high yaw noise and low speed noise; trucks
the reverse.

The **CV model** (constant velocity, linear, `F` is a constant matrix) is implemented
behind the same interface so the M4 ablation is a config flag, not a code change.

### 5.3 Yaw handling

Every place yaw appears:

- Normalize to `(−π, π]` after prediction and after update.
- The innovation `y = z − h(x)` must use an **angular difference**, not subtraction.
  `atan2(sin(a−b), cos(a−b))`.
- Detection yaw is extracted from the global quaternion as the rotation about the
  z-axis. nuScenes boxes are gravity-aligned, so yaw is the only meaningful rotation.
- Optional refinement: some detectors emit yaw flipped by π for symmetric objects.
  If the measured yaw is closer to `x.yaw + π` than to `x.yaw`, flip the measurement.
  This is worth an ablation row.

### 5.4 Measurement model

```
z = [ px, py, pz, yaw ]ᵀ                       (base)
z = [ px, py, pz, yaw, vx, vy ]ᵀ               (when detection velocity is valid)
```

Position and yaw are observed directly → linear `H`. Velocity is **not**:
`vx = v·cos(yaw)`, `vy = v·sin(yaw)` is nonlinear, so `H` becomes a Jacobian for
those two rows.

Note for interviews: even with a purely linear measurement model this is an EKF,
because the *CTRV prediction* is nonlinear. The linearization lives in the predict
step. People get this backwards constantly.

`R` is per-class, tuned from detector error statistics measured at M3 (compute the
empirical error of matched detections against ground truth once, and set `R` from it,
rather than guessing).

### 5.5 Data association

Runs per class — a car never competes with a pedestrian for an assignment. This makes
the cost matrices small and fast and is also correct.

**Cost functions** behind one interface, selectable by config:

1. **Mahalanobis distance** using the innovation covariance
   `S = H P Hᵀ + R`, `d² = yᵀ S⁻¹ y` over the position sub-block.
   Gate: `d² > χ²(0.99, df=3) = 11.34` → forbidden.
2. **3D IoU** — rotated BEV polygon intersection (Sutherland–Hodgman clipping of two
   rotated rectangles, then shoelace area) × vertical overlap. Cost = `1 − IoU`.
   Gate: `IoU < 0.1` → forbidden.
3. **Euclidean center distance** (cheap baseline for the ablation table).
   Gate: class-dependent, e.g. 2 m pedestrian / 4 m car / 6 m truck.

Gating happens **before** the assignment, by writing a large sentinel into forbidden
cells. This is both a correctness and a speed decision.

**Assignment: Hungarian algorithm (Jonker-Volgenant / Kuhn-Munkres), implemented by
hand.** Not pulled from a library. Roughly 200 lines, self-contained, `O(n³)`, and it
is one of the more credible things you can point at in a C++ interview. It is
unit-tested against brute-force enumeration on all matrices up to 6×6 and against
known-optimal fixtures.

### 5.6 Track lifecycle

A three-state machine per track:

```
       (unmatched detection)
              │
              ▼
        ┌──────────┐  hits ≥ min_hits   ┌───────────┐
        │TENTATIVE │ ─────────────────▶ │ CONFIRMED │
        └──────────┘                    └───────────┘
              │                                │
   misses ≥ 1 │                                │ misses ≥ max_age
              ▼                                ▼
        ┌──────────┐                    ┌───────────┐
        │  DEAD    │ ◀──────────────────│   DEAD    │
        └──────────┘                    └───────────┘
```

- Only `CONFIRMED` tracks are written to the submission.
- `min_hits` and `max_age` are **per class**. At 2 Hz, `max_age = 3` is 1.5 seconds of
  coasting — quite a lot. Pedestrians (erratic, frequently occluded) and trucks
  (large, stable, high detection recall) want different values.
- Track score for the submission: running average of associated detection scores.
  AMOTA integrates over recall, so this score directly controls where each track lands
  on the recall curve. **Getting the track score wrong is the most common reason a
  correct tracker reports a bad AMOTA.** Ablate: max score vs mean score vs last score.
- Detection score threshold before the tracker sees anything: per class, ablated.

---

## 6. Evaluation

### 6.1 Metrics

Produced by `nuscenes-devkit`'s official tracking eval
(`nuscenes.eval.tracking.evaluate`), which is the same code the leaderboard runs.

- **AMOTA** — primary. MOTA averaged over recall thresholds, recall-normalized
  (MOTAR). This is the number on the resume.
- **AMOTP** — average localization error of matched tracks.
- **IDS** — identity switches.
- **FRAG** — fragmentations.
- **TID / LGD** — track initialization duration / longest gap duration. Directly
  sensitive to `min_hits` and `max_age`, so they're the diagnostic for lifecycle
  tuning.

Matching is by center distance with a 2 m threshold.

### 6.2 Baseline comparison

**Published baseline, from `configs/nusc/README.md` in the CenterPoint repo:**

| | AMOTA ↑ | AMOTP ↓ | Tracking time |
|---|---|---|---|
| CenterPoint tracker, `centerpoint_voxel_1024` detections, val | **63.7** | 0.606 | 1 ms/frame |

Two caveats to state honestly in the README. That 63.7 was measured on the
`voxel_1024` detections; we are using the newer `0075voxel_fix_bn_z` detections, which
are *better* (59.6 vs 56.4 mAP), so a like-for-like comparison isn't exact — note the
mismatch rather than papering over it. And their tracker is greedy closest-point
matching that leans on the detector's predicted velocity, not an EKF, so a lower
number from us is a legitimate result, not a failure.

**A reference implementation exists and should be used.** `tools/nusc_tracking/pub_test.py`
in the CenterPoint repo consumes the same detection JSON via `--checkpoint` and
produces a submission. When our AMOTA comes out wrong at M3, running theirs on the same
file and diffing the outputs isolates whether the bug is in our tracker or our
eval harness. Cheapest debugging asset in the project.

**Do not write numbers from memory — pull them from the source and cite it.**

The claim the project supports is: *"reproduced a published tracking baseline on
nuScenes val to within X AMOTA, then ablated the design choices."* That is a stronger
and more honest claim than "state of the art," and it's the one that survives
scrutiny.

### 6.3 Ablation table (the actual differentiator)

Almost every tracker repo on GitHub ships one configuration and no table. The table is
what makes this look like engineering rather than a tutorial follow-along.

| # | Motion | Cost | Per-class params | Track score | AMOTA | AMOTP | IDS | FRAG |
|---|---|---|---|---|---|---|---|---|
| 1 | CV | Euclidean | no | mean | | | | |
| 2 | CV | Mahalanobis | no | mean | | | | |
| 3 | CTRV | Mahalanobis | no | mean | | | | |
| 4 | CTRV | Mahalanobis | yes | mean | | | | |
| 5 | CTRV | 3D IoU | yes | mean | | | | |
| 6 | CTRV | Mahalanobis | yes | max | | | | |

Each row is one config file and one command. The table is generated by a script, not
by hand — `tools/run_ablation.py` reads a list of configs, runs each, scores each,
emits Markdown.

---

## 7. Repository layout

```
mot3d-nuscenes/
├── CMakeLists.txt
├── README.md                  # the deliverable — written last, at M7
├── LICENSE                    # MIT
├── .clang-format
├── .clang-tidy
├── .github/workflows/ci.yml
├── docker/
│   ├── Dockerfile             # ubuntu:22.04 + build deps
│   └── Dockerfile.ros         # ros:humble + build deps
├── docs/
│   ├── SPEC.md                # this file
│   ├── RESULTS.md
│   └── FUTURE_WORK.md
├── configs/
│   ├── default.yaml
│   └── ablation/*.yaml
├── include/mot3d/             # public headers
│   ├── geometry/              # SE3, Quaternion, Box3D, iou3d
│   ├── filter/                # KalmanFilter, MotionModel, CTRV, CV
│   ├── assoc/                 # CostFunction, Mahalanobis, IoU, Hungarian
│   ├── track/                 # Track, TrackState, TrackManager
│   ├── io/                    # DetectionLoader, SubmissionWriter, NuScenesMeta
│   └── config/                # Config, per-class params
├── src/                       # implementations, mirrors include/
├── apps/
│   └── track_nuscenes.cpp     # CLI: config in, submission JSON out
├── ros2/mot3d_ros/            # separate ament package, links the core lib
├── tests/                     # GoogleTest, mirrors module structure
├── bench/                     # timing harness
└── tools/                     # Python: eval driver, ablation runner, plots
```

**The core library has no ROS dependency.** ROS 2 is a thin wrapper in a separate
package. This is how it's done in production and it's an easy thing to point at.

---

## 8. Tech stack

| Concern | Choice | Note |
|---|---|---|
| Language | C++17 | Not 20 — ROS 2 Humble's default toolchain |
| Build | CMake ≥ 3.16, Ninja | |
| Linear algebra | Eigen **3.4.0, pinned via FetchContent** | Homebrew ships Eigen 5.x; Ubuntu 22.04 (CI + ROS container) ships 3.4. Pinning makes local, CI, and Docker byte-identical. Fixed-size types in hot paths. |
| JSON | nlohmann/json | Fetched via CMake FetchContent |
| Config | yaml-cpp | |
| Tests | GoogleTest | via FetchContent |
| Logging | spdlog | |
| Container | Docker, Ubuntu 22.04 | Reproducibility is part of the deliverable |
| CI | GitHub Actions | build + ctest + clang-format check on every push |
| ROS | ROS 2 Humble | rviz2 for visualization |
| Eval / tooling | Python 3.10, nuscenes-devkit | Only place Python appears |
| Profiling | **Xcode Instruments** (Time Profiler) + in-process stage timers | `perf` needs PMU access, unavailable on macOS and inside containers on macOS. The core library builds natively on the host precisely so it can be profiled there. |

**Dependency policy:** anything fetched by CMake at configure time (json, gtest, yaml,
spdlog) is fine. Eigen and ROS come from the system/container. No vendored source
trees, no git submodules.

---

## 9. Milestones

Each milestone ends with a commit, a green CI run, and a tag. Nothing moves forward
until CI is green.

### M0 — Skeleton
CMake project that builds an empty library and a hello-world test. Dockerfile. GitHub
Actions running build + ctest + format check. `v1.0-mini` downloaded and extracted.
`SPEC.md` committed.
**Done when:** a green CI badge on a repo containing zero tracker code.

### M1 — Data layer
Detection JSON parsing. nuScenes metadata loading. `SE3`/quaternion types and the
sensor→ego→global chain. Submission writer producing a file the devkit will accept.
**Done when:** a round-trip transform test passes to 1e-9; the writer emits an empty
but schema-valid submission that `nuscenes-devkit` loads without error. Proving the
output format *before* the tracker exists removes an entire class of debugging later.

### M2 — Tracker core
EKF predict/update, CTRV + CV, all three cost functions, hand-written Hungarian,
lifecycle FSM.
**Done when these tests pass:** analytic Jacobian matches finite difference; a
constant-velocity synthetic object is tracked 40 frames with zero ID switches; two
objects crossing paths do not swap IDs; yaw wrapping across ±π is continuous; the
`yaw_rate ≈ 0` branch produces no NaN; Hungarian matches brute force on all matrices
up to 6×6.

### M3 — First real number
Run over all of val, emit submission, score with the official eval.
**Done when:** an AMOTA exists, whatever it is. It will be bad. Getting from bad to
reasonable *is the project* — budget more time here than anywhere else, and keep a
running log in `docs/RESULTS.md` of what you changed and what it did to the number.
**This milestone is the gate for the resume bullet.** Until it produces a number, the
resume line stays in present tense.

### M4 — Ablation table
Six configs, scripted, one Markdown table.
**Done when:** `tools/run_ablation.py` regenerates the table from scratch.

### M5 — Profiling
Per-stage timers (predict / gating / cost matrix / assignment / update / lifecycle),
p50 and p95 ms per frame, call tree from Instruments' Time Profiler (run against the
native macOS build, not the container). Then **one** optimization pass: fixed-size Eigen
types, preallocated track pools, `reserve()` on hot vectors, avoid recomputing `S⁻¹`.
**Done when:** a before/after table exists. **Keep the "before" number** — the delta is
the story, not the final figure.

### M6 — ROS 2 + rviz
`mot3d_ros` node subscribing to detections, publishing tracks as `MarkerArray` with
stable per-ID colors and ID text labels. Rosbag playback of a val scene, screen
recording in the README.
**Done when:** a GIF of tracks following vehicles in rviz2 is in the README.

### M7 — README
Results vs baseline. Ablation table. Latency table. Architecture diagram.
**Three or four failure cases** with images and a written explanation of the cause.
**Done when:** someone who has never seen the repo can understand what it does, what
it scores, and where it breaks, in under two minutes.

---

## 10. Risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| ~~Detection result file link is dead~~ | **RESOLVED** | Checked before M0 finished. Live on MIT OneDrive (not Google Drive), no login required, 345 MB JSON downloaded and format verified |
| Disk fills up | Medium | Metadata-only for scoring; download at most 2 sensor blobs |
| Scope creep into learned association | High | This document. Re-read §3 |
| AMOTA stuck low and cause unclear | Medium | Per-class metric breakdown; sanity-check on a single scene with ground-truth boxes injected as "detections" — a perfect input must yield near-perfect AMOTA, and if it doesn't, the bug is in I/O or lifecycle, not the filter |
| ROS 2 environment fights the core build | Low | Two Dockerfiles, core library has no ROS dependency |

---

## 11. Guardrails

**Do the failure analysis yourself.** Claude Code will write the EKF fine. Looking at
where tracking breaks and understanding *why* is the part that becomes the interview
answer, and it cannot be generated.

**Only claim numbers you ran.** Every figure in the README traces to a command in the
repo that reproduces it.

**Commit at every step.** The commit history is part of the artifact. A repo with 60
meaningful commits over four weeks reads very differently from one with three.

---

## 12. Resume output

Once M3–M5 land, the bullets write themselves:

- C++17 3D multi-object tracker (EKF + CTRV, Hungarian association, per-class
  lifecycle) on nuScenes val, reproducing a published baseline at **X AMOTA** using
  the same detections.
- Ablated motion model, association cost, and lifecycle policy across six
  configurations; **[finding]**.
- Profiled and optimized the pipeline from **A ms to B ms** per frame.
- Packaged as a ROS 2 node with rviz2 visualization; Dockerized, CI-tested, MIT
  licensed.
