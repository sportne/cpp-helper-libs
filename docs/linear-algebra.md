# Linear Algebra Module

## What This Module Is For

`libs/linear_algebra` provides immutable vector and matrix math utilities used across the
repository.

Public umbrella include:

```cpp
#include "cpp_helper_libs/linear_algebra/linear_algebra.hpp"
```

## Core Types

### `Vector3`

General 3D Cartesian vector.

- Stores `(x, y, z)` components.
- Supports basic vector algebra (`+`, `-`, dot, cross, magnitude).
- Can be normalized into `UnitVector3` when non-zero.

Use this when a value is not guaranteed to have unit length.

### `UnitVector3`

3D direction vector constrained to magnitude 1.

- Must be constructed through checked factories (`from_components`, `from_vector`).
- Keeps direction-only semantics explicit.
- Provides central-angle operations for geometry code.

Use this when you represent a pure direction (for example, radial directions on a unit sphere).

### `Matrix3`

Fixed-size immutable `3x3` matrix.

- Fast, explicit math for small systems.
- Supports arithmetic, matrix products, norms, determinant, inverse.
- Includes solver/decomposition support:
  - `lu()`
  - `qr()`
  - `cholesky()`
  - `solve(...)`

Use this when shape is known to be 3x3.

### `Matrix`

General immutable dynamic matrix.

- Stores row/column count plus row-major values.
- Supports shape-safe arithmetic and multiplication.
- Returns `std::nullopt` on shape mismatch (or invalid operations like Hadamard divide by zero).

Use this for non-3x3 workloads.

### Decomposition Result Types

- `LU3`: stores `L`, `U`, permutation, parity
- `QR3`: stores `Q`, `R`
- `Cholesky3`: stores `L`

These types keep decomposition data lightweight and reconstruct `Matrix3` on demand.

## Design Principles

- **Immutability first:** operations return new values rather than mutating state.
- **Shape safety:** dynamic matrix operations fail gracefully via `std::nullopt` when dimensions do
  not match.
- **Deterministic random helpers:** seeded random constructors support stable tests/reproducibility.
- **Numerical caution:** tolerance checks are used in decomposition/solve code for near-singular
  cases.

## Minimal Example

```cpp
#include "cpp_helper_libs/linear_algebra/linear_algebra.hpp"

using cpp_helper_libs::linear_algebra::Matrix3;
using cpp_helper_libs::linear_algebra::Vector3;
using cpp_helper_libs::linear_algebra::unit_normal;

const auto n = unit_normal(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0));
// n is approximately (0, 0, 1)

const Matrix3 a(4.0, 1.0, 2.0,
                1.0, 5.0, 1.0,
                2.0, 1.0, 3.0);
const auto x = a.solve(Vector3(7.0, 8.0, 5.0));
const double det = a.determinant(); // 37.0
```

## Where To Read Next

- Public API declarations:
  - `libs/linear_algebra/include/cpp_helper_libs/linear_algebra/`
- Implementations:
  - `libs/linear_algebra/src/`
- Behavioral examples:
  - `libs/linear_algebra/tests/`
