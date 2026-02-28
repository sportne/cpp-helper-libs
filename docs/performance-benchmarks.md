# Performance Benchmarks

## Purpose

This repository includes local performance tooling for `libs/spherical_geometry`.
It supports two complementary goals:

- guardrails: catch regressions before they merge
- optimization guidance: identify hotspots and where to focus tuning effort

This workflow is local-only and is not part of CI quality gates.

## Prerequisites

Initialize submodules:

```bash
git submodule update --init --recursive
```

Required submodule:
- `third_party/googlebenchmark`

Optional Linux tools for deeper hotspot analysis:
- `perf` (`linux-perf`)

## Quick Commands

Baseline regression workflow:

```bash
cmake --workflow --preset benchmark
cmake --workflow --preset benchmark-baseline
cmake --workflow --preset benchmark-compare
```

Hotspot-analysis workflow:

```bash
cmake --workflow --preset benchmark-hotspots
cmake --workflow --preset benchmark-perfstat
cmake --workflow --preset benchmark-profile
```

Equivalent `make` aliases:

```bash
make bench
make bench-baseline
make bench-compare
make bench-hotspots
make bench-perfstat
make bench-profile
```

## Outputs

Regression and baseline data:
- latest run JSON: `build/clang-benchmark/benchmarks/spherical_geometry/latest.json`
- local baseline JSON: `docs/benchmarks/spherical_geometry-baseline.local.json`

Hotspot-analysis artifacts:
- hotspot benchmark JSON: `build/clang-benchmark/benchmarks/spherical_geometry/analysis/hotspots.json`
- hotspot summary: `build/clang-benchmark/benchmarks/spherical_geometry/analysis/hotspots-summary.md`
- perf stat CSV: `build/clang-benchmark/benchmarks/spherical_geometry/analysis/perfstat/perfstat.csv`
- perf stat raw logs: `build/clang-benchmark/benchmarks/spherical_geometry/analysis/perfstat/raw/`
- perf profile data: `build/clang-benchmark/benchmarks/spherical_geometry/analysis/profile/perf.data`
- perf profile report: `build/clang-benchmark/benchmarks/spherical_geometry/analysis/profile/perf-report.txt`

The baseline file is intentionally machine-local and ignored by git.

## How To Use the Data

1. Start with `benchmark-hotspots`.
2. Open `hotspots-summary.md` and pick the slowest benchmark entries first.
3. Run `benchmark-perfstat` to classify bottlenecks:
   - high `cache-misses`: memory-access locality issue
   - high `branch-misses`: branch-predictability issue
   - high `cycles` with lower `instructions`: latency/memory pressure
   - if hardware counters are unavailable (common in WSL/VMs), the runner automatically falls back to software counters (`cpu-clock`, `task-clock`, `context-switches`, `page-faults`)
4. Run `benchmark-profile` to identify concrete functions in hot stacks.
5. Make one focused optimization change and rerun the same sequence.

## Notes

- Benchmarks use size sweeps (`small`, `medium`, `large`) so scaling trends are visible.
- `benchmark-compare` fails if any median `cpu_time` regresses by more than `10%`.
- If `perf` is unavailable or blocked by permissions, perf-based targets fail with actionable errors.
- `benchmark-perfstat` first requests hardware events (`cycles,instructions,cache-misses,branch-misses`) and automatically retries each benchmark with software events if needed.
