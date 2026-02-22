# Spherical Geometry Module

## What This Module Is For

`libs/spherical_geometry` is for geometry on the **surface** of a unit sphere.
Think "Earth-like globe math" where points are directions from the center.

If you are new to this topic, keep this mental model:

- A point on the sphere is a unit vector from the origin (`UnitVector3`).
- Distance along the sphere is measured by a **central angle** (in radians or degrees).
- Curves are paths that stay on the sphere surface.
- Shapes are regions bounded by those curves.

Public umbrella include:

```cpp
#include "cpp_helper_libs/spherical_geometry/spherical_geometry.hpp"
```

## Core Concepts

### Coordinate

`Coordinate` stores latitude/longitude and converts to/from `UnitVector3`.

- Latitude is clamped to `[-90, +90]` degrees.
- Longitude is wrapped to `[-180, +180)` degrees.

This is the most human-friendly type in the module.

### SphericalRay

`SphericalRay` is a local frame at a point on the sphere:

- `radial`: points outward from origin through the surface point
- `tangent`: "forward" direction along the surface
- `normal`: perpendicular direction, with `normal = radial x tangent`

Use it when you need both a location and orientation.

### SphericalCurve

`SphericalCurve` is the shared interface for curve types:

- start/end radial and start/end ray
- arc length
- intersection queries with other curves

Concrete curves:

- `MinorArc`: great-circle segment with sweep in `(0, pi)`
- `MajorArc`: great-circle segment with sweep in `[pi, 2*pi)`
- `SmallArc`: small-circle segment (defined by center + radius + direction)
- `ZeroLengthCurve`: degenerate "point-curve" that still satisfies the interface

### SphericalShape

`SphericalShape` is the shared interface for regions:

- contains point? (inclusive/exclusive)
- boundary intersects curve? (inclusive/exclusive)

Concrete shapes:

- `SphericalPolygon`: closed non-self-intersecting polygon of minor-arc edges
- `SphericalCircle`: small-circle region
- `SphericalEllipse`: two-foci constant-sum-angle region

## Intersection Model

Curve/curve intersections return `CurveIntersection` records.

Kinds:

- `Point`: ordinary crossing point
- `EndpointTouch`: only touches at one or more endpoints
- `OverlapSegment`: curves overlap on a non-zero segment

Each returned point has `CurveLocation` metadata:

- normalized `parameter` in `[0, 1]`
- `at_start` and `at_end` flags

## Tolerant vs Exact APIs

Most operations have two variants:

- tolerant (default): robust against floating-point noise
- exact: strict comparisons

Examples:

- `curve.intersections_with(...)` vs `curve.intersections_with_exact(...)`
- `shape.contains_inclusive(...)` vs `shape.contains_inclusive_exact(...)`
- `shape.boundary_intersects_exclusive(...)` vs `shape.boundary_intersects_exclusive_exact(...)`

When in doubt, start with tolerant variants.

## Important Design Notes

- `SphericalPolygon` assumes vertices are ordered and boundary edges are valid minor arcs.
- `SphericalCircle` internally stores boundary as two `SmallArc` halves for robust intersections.
- `SphericalEllipse` uses exact sum-angle containment but approximates boundary with many `MinorArc`
  segments for practical boundary-intersection checks.
- `ZeroLengthCurve` exists so point-like geometry can still use the same curve APIs.

## Minimal Example

```cpp
#include "cpp_helper_libs/spherical_geometry/spherical_geometry.hpp"

using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::spherical_geometry::Coordinate;
using cpp_helper_libs::spherical_geometry::MinorArc;

const auto x = UnitVector3::from_components(1.0, 0.0, 0.0).value();
const auto y = UnitVector3::from_components(0.0, 1.0, 0.0).value();

const MinorArc arc = MinorArc::from_endpoints(x, y).value();
const Coordinate end = Coordinate::from_radial(arc.end_radial());

// end.longitude() is approximately +90 degrees
```

## Where To Read Next

- API declarations:
  - `libs/spherical_geometry/include/cpp_helper_libs/spherical_geometry/`
- Implementation details:
  - `libs/spherical_geometry/src/`
- Usage examples and behavior checks:
  - `libs/spherical_geometry/tests/`
