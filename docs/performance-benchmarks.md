# Performance Benchmarks

## Purpose

This repository includes a local benchmark workflow for `libs/spherical_geometry`.
It is designed for:

- measuring runtime changes while developing
- recording a baseline on your machine
- comparing later runs against that baseline

This workflow is local-only and is not part of CI quality gates.

## Prerequisites

- Initialize submodules:

```bash
git submodule update --init --recursive
```

This must include:
- `third_party/googlebenchmark`

## Run Benchmarks

Run benchmarks and write JSON output to:
`build/clang-benchmark/benchmarks/spherical_geometry/latest.json`

```bash
cmake --workflow --preset benchmark
```

Equivalent shortcut:

```bash
make bench
```

The runner uses:
- repeated samples
- aggregate-only reporting
- median CPU time for comparisons
- a runtime target tuned for roughly 20 seconds total on a typical developer machine

## Record a Local Baseline

Create or refresh the local baseline file:
`docs/benchmarks/spherical_geometry-baseline.local.json`

```bash
cmake --workflow --preset benchmark-baseline
```

Equivalent shortcut:

```bash
make bench-baseline
```

The baseline file is intentionally machine-local and ignored by git.

## Compare Against Baseline

Run benchmarks and fail if any benchmark regresses by more than `10%`
for median `cpu_time`:

```bash
cmake --workflow --preset benchmark-compare
```

Equivalent shortcut:

```bash
make bench-compare
```

If the baseline file is missing, the compare step fails and tells you to run
`benchmark-baseline` first.
