# mot3d_nuscenes

3D multi-object tracking on the nuScenes dataset. C++17, EKF with a CTRV motion
model, Hungarian data association, evaluated with the official nuScenes tracking
metrics.

**Status: in development.** No results yet.

See [docs/SPEC.md](docs/SPEC.md) for the full system specification.

## Building

```bash
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

## License

MIT
