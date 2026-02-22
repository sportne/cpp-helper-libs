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
  /**
   * @brief Create a zero-length curve anchored at a provided ray.
   */
  static ZeroLengthCurve at(const SphericalRay &ray) noexcept;

  /**
   * @brief Create a zero-length curve at a point radial with inferred tangent frame.
   *
   * @return Point-curve when a valid orthogonal tangent can be produced, else `std::nullopt`.
   */
  static std::optional<ZeroLengthCurve>
  at_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &radial) noexcept;

  /**
   * @brief Unit radial of the only point on the curve.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 start_radial() const noexcept override;
  /**
   * @brief Unit radial of the only point on the curve.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 end_radial() const noexcept override;

  /**
   * @brief Oriented frame at the only point on the curve.
   */
  SphericalRay start_ray() const noexcept override;
  /**
   * @brief Oriented frame at the only point on the curve.
   */
  SphericalRay end_ray() const noexcept override;

  /**
   * @brief Always returns zero angle.
   */
  cpp_helper_libs::quantities::Angle length() const noexcept override;

protected:
  /**
   * @brief Support-axis value used by shared intersection plumbing.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 support_axis() const noexcept override;
  /**
   * @brief Support-plane constant used by shared intersection plumbing.
   */
  double support_constant() const noexcept override;
  /**
   * @brief Signed sweep used by shared intersection plumbing.
   *
   * Always `0` for this degenerate curve.
   */
  double signed_sweep_radians() const noexcept override;
  /**
   * @brief Locate candidate point; success only for the represented radial.
   */
  std::optional<CurveLocation>
  locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
               bool exact) const noexcept override;
  /**
   * @brief Parameter evaluation, independent of parameter value.
   */
  cpp_helper_libs::linear_algebra::UnitVector3
  point_at_parameter(double parameter) const noexcept override;
  /**
   * @brief Report degenerate curve semantics.
   *
   * Always `true` for @ref ZeroLengthCurve.
   */
  bool is_zero_length_curve() const noexcept override;

private:
  explicit ZeroLengthCurve(const SphericalRay &ray) noexcept : ray_(ray) {}

  /// Frame anchored at the single point represented by this degenerate curve.
  SphericalRay ray_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_ZERO_LENGTH_CURVE_HPP
