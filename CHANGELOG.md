# Changelog

All notable changes to this project are documented in this file. The format is
based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this
project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.0] - 2026-05-07

### Added

- `--error_metric` option to choose the optimization error metric:
  `point_to_plane` (default; requires normals on the fixed cloud) or
  `point_to_point` (no normals required).
- `--sigma_mad_factor` option to control MAD-based outlier rejection
  (default `5`, previously hard-coded to `3`). Set to `0` to disable.
- Voxel-stratified sampling of correspondences via
  `--max_correspondences_per_voxel` (default `20`). Replaces purely random
  sampling and ensures every translation-grid voxel that contains fixed-cloud
  data is constrained by at least one correspondence.
- Per-derivative weighting of zero observations: the four values in
  `--weights "f,fx/fy/fz,fxy/fxz/fyz,fxyz"` are now each applied to their
  respective derivative classes. Previously only the first value was applied
  to all unknowns.
- Solver-failure messages now report iteration count, residual error,
  tolerance, and problem size to aid debugging.
- Input validation: `--weights` must contain exactly four values,
  `--max_correspondences_per_voxel` must be `> 0`, `--sigma_mad_factor`
  must be `>= 0`. Voxel-stratified sampling raises a clear error if every
  fixed-cloud point lies outside the moving cloud's translation grid.

### Changed

- Default MAD rejection factor is now `5` (configurable) instead of `3`
  (hard-coded).

### Removed

- `--num_correspondences` flag, replaced by
  `--max_correspondences_per_voxel` with voxel-stratified sampling.

### Fixed

- Windows CI: `fmt` and `spdlog` are now installed via conda alongside
  PDAL to avoid the version mismatch between vcpkg's `fmt` and the
  conda-resident `fmt` that PDAL's include path transitively exposes.
  The MSVC build also adds `/utf-8`, required by the `static_assert` in
  `fmt 12`.

## [1.1.0] - 2025-09-22

- Windows CI/CD support with MSVC and vcpkg.
- AppImage distribution for Linux.
- `nonrigid-icp-transform`: chunk-based point-cloud processing to
  reduce peak memory usage.
- Switched logging from `spdlog` to `fmt`.

## [1.0.0]

- Initial release.

[1.2.0]: https://github.com/AIT-Assistive-Autonomous-Systems/3D_nonrigid_ICP/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/AIT-Assistive-Autonomous-Systems/3D_nonrigid_ICP/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/AIT-Assistive-Autonomous-Systems/3D_nonrigid_ICP/releases/tag/v1.0.0
