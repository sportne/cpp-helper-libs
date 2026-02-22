# Quantities Module

## What This Module Is For

`libs/quantities` provides strongly-typed scalar measurement classes with unit-aware conversion.

Public umbrella include:

```cpp
#include "cpp_helper_libs/quantities/quantities.hpp"
```

## Why Use It Instead Of Plain `double`

Using raw numbers makes unit mistakes easy (for example, mixing miles and meters).
This module makes those mistakes harder by:

- naming the physical quantity type (`Length`, `Time`, `Speed`, etc.)
- requiring explicit units at construction/conversion
- storing a canonical internal unit for consistent arithmetic/comparison

## Core Types

- `Length` (canonical: meters)
- `Time` (canonical: seconds)
- `Speed` (canonical: meters per second)
- `Mass` (canonical: kilograms)
- `Force` (canonical: newtons)
- `Acceleration` (canonical: meters per second squared)
- `Angle` (canonical: radians)

All of them are thin wrappers around `QuantityBase<T>`.

## Common Behavior

Each quantity type supports:

- factory methods per unit (`Length::miles(...)`, `Time::milliseconds(...)`, ...)
- conversion with `in(Unit::...)`
- exact comparisons and ordering on canonical values
- addition/subtraction with same quantity type
- scalar multiply/divide through `QuantityBase`
- stable hashing (`+0.0` and `-0.0` hash the same)

## Angle-Specific Utilities

`Angle` also supports wrapping helpers:

- `bound_zero_to_two_pi()` gives `[0, 2*pi)`
- `bound_negative_pi_to_pi()` gives `[-pi, pi)`

These are useful in orientation/geometry code.

## Minimal Example

```cpp
#include "cpp_helper_libs/quantities/quantities.hpp"

using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::quantities::Length;
using cpp_helper_libs::quantities::Speed;
using cpp_helper_libs::quantities::Time;

const auto trip_distance = Length::miles(1.0);
const double meters = trip_distance.in(Length::Unit::Meter); // 1609.344

const auto duration = Time::seconds(80.0);
const auto velocity = Speed::meters_per_second(
    trip_distance.in(Length::Unit::Meter) / duration.in(Time::Unit::Second));

const auto heading = Angle::degrees(450.0).bound_zero_to_two_pi(); // 90 degrees equivalent
```

## Design Principles

- **Strong typing over convenience:** quantity classes prevent accidental unit mixing.
- **Canonical storage:** each type converts to one internal base unit for stable arithmetic.
- **Explicit conversion boundaries:** all unit conversion is centralized in each type's `to_raw` and
  `in` functions.
- **Predictable failure mode:** invalid enum inputs throw `std::invalid_argument`.

## Where To Read Next

- Public API declarations:
  - `libs/quantities/include/cpp_helper_libs/quantities/`
- Implementations:
  - `libs/quantities/src/`
- Behavioral examples:
  - `libs/quantities/tests/`
