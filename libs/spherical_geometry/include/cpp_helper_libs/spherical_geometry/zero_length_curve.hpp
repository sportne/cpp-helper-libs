// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_ZERO_LENGTH_CURVE_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_ZERO_LENGTH_CURVE_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_curve.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::spherical_geometry {

/**
 * @brief Degenerate point-like curve implementing SphericalCurve.
 */
class ZeroLengthCurve final : public SphericalCurve {
public:
  static ZeroLengthCurve at(const SphericalRay &ray) noexcept;

  static std::optional<ZeroLengthCurve>
  at_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &radial) noexcept;

  cpp_helper_libs::linear_algebra::UnitVector3 start_radial() const noexcept override;
  cpp_helper_libs::linear_algebra::UnitVector3 end_radial() const noexcept override;

  SphericalRay start_ray() const noexcept override;
  SphericalRay end_ray() const noexcept override;

  cpp_helper_libs::quantities::Angle length() const noexcept override;

protected:
  cpp_helper_libs::linear_algebra::UnitVector3 support_axis() const noexcept override;
  double support_constant() const noexcept override;
  double signed_sweep_radians() const noexcept override;
  std::optional<CurveLocation>
  locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
               bool exact) const noexcept override;
  cpp_helper_libs::linear_algebra::UnitVector3
  point_at_parameter(double parameter) const noexcept override;
  bool is_zero_length_curve() const noexcept override;

private:
  explicit ZeroLengthCurve(const SphericalRay &ray) noexcept : ray_(ray) {}

  /// Frame anchored at the single point represented by this degenerate curve.
  SphericalRay ray_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_ZERO_LENGTH_CURVE_HPP
