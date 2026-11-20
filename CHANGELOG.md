# Changelog

All notable changes to this project are listed here, one section per version tag.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions
follow [Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-11-20

M2: the tracker itself, tested on synthetic data.

### Added

- CTRV motion model with an analytic Jacobian, plus a constant-velocity (CV) model, selected
  through a `MotionModel` switch in the filter.
- Extended Kalman filter over `[x, y, z, yaw, speed, yaw_rate]`: predict, update with the
  Joseph-form covariance, and yaw wrapping.
- Association costs: Mahalanobis, Euclidean, and 3D IoU (`1 - BoxIou`), with rotated-box overlap
  computed by polygon clipping.
- Hand-written Hungarian assignment with gating by `max_cost`.
- Track lifecycle: birth from unmatched detections, reporting after `min_hits`, deletion after
  `max_misses`.
- `Tracker`: per-frame predict, cost, match, update, birth, deletion and reporting, with
  class-aware matching.
- Pre-commit hook running the pinned clang-format; CI builds with `-Werror`.
- CI job running every test under AddressSanitizer and UndefinedBehaviorSanitizer.

### Changed

- The state Jacobian is named `jac` everywhere; the CV tests reuse the numeric Jacobian helper.

## [0.1.0] - 2026-10-26

M1: data in, results out.

### Added

- Detection loader for the CenterPoint nuScenes result file (drops the 3 non-tracking classes,
  takes yaw from the quaternion).
- Scene loader that orders frames by walking each scene's sample chain.
- Submission writer for the official nuScenes tracking format.
- Geometry module: poses, and yaw and quaternion conversion.
- Check that the official nuScenes scorer accepts a valid submission.
- CI on Ubuntu 22.04 (build, tests, clang-format) with pinned Eigen 3.4.0, nlohmann/json,
  GoogleTest and clang-format.

[0.2.0]: https://github.com/Moin-Palekar/mot3d-nuscenes/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/Moin-Palekar/mot3d-nuscenes/releases/tag/v0.1.0