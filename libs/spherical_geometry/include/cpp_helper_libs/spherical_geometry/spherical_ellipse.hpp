// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_ELLIPSE_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_ELLIPSE_HPP

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_shape.hpp"

namespace cpp_helper_libs::spherical_geometry {

class SphericalCurve;

/**
 * @brief Spherical ellipse defined by two foci and a constant sum-angle boundary.
 */
class SphericalEllipse final : public SphericalShape {
public:
  /// Default number of boundary segments used to approximate ellipse perimeter operations.
  static constexpr std::size_t kDefaultBoundarySegmentCount = 360U;

  /**
   * @brief Create a spherical ellipse from two foci and boundary sum angle.
   *
   * A point lies on the boundary when
   * `angle(focus_one, point) + angle(focus_two, point) == boundary_sum`.
   *
   * @param focus_one First focus direction.
   * @param focus_two Second focus direction.
   * @param boundary_sum Constant boundary sum of focus angles.
   * @param boundary_segment_count Number of boundary segments used for polygonal approximation.
   * @return Ellipse on valid non-degenerate inputs, else `std::nullopt`.
   */
  static std::optional<SphericalEllipse> from_foci_and_boundary_sum(
      const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
      const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
      cpp_helper_libs::quantities::Angle boundary_sum,
      std::size_t boundary_segment_count = kDefaultBoundarySegmentCount) noexcept;

  /**
   * @brief First ellipse focus direction.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 focus_one() const noexcept { return focus_one_; }
  /**
   * @brief Second ellipse focus direction.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 focus_two() const noexcept { return focus_two_; }
  /**
   * @brief Boundary sum-angle constant.
   */
  cpp_helper_libs::quantities::Angle boundary_sum() const noexcept { return boundary_sum_; }
  /**
   * @brief Boundary segmentation density used internally for intersection operations.
   */
  std::size_t boundary_segment_count() const noexcept { return boundary_segment_count_; }

  /**
   * @brief Inclusive containment query with tolerant numeric policy.
   */
  bool contains_inclusive(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;
  /**
   * @brief Exclusive containment query with tolerant numeric policy.
   */
  bool contains_exclusive(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;
  /**
   * @brief Inclusive containment query with exact numeric policy.
   */
  bool contains_inclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;
  /**
   * @brief Exclusive containment query with exact numeric policy.
   */
  bool contains_exclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;

  /**
   * @brief Inclusive boundary intersection query with tolerant numeric policy.
   */
  bool boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept override;
  /**
   * @brief Exclusive boundary intersection query with tolerant numeric policy.
   */
  bool boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept override;
  /**
   * @brief Inclusive boundary intersection query with exact numeric policy.
   */
  bool boundary_intersects_inclusive_exact(const SphericalCurve &curve) const noexcept override;
  /**
   * @brief Exclusive boundary intersection query with exact numeric policy.
   */
  bool boundary_intersects_exclusive_exact(const SphericalCurve &curve) const noexcept override;

private:
  SphericalEllipse(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                   const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                   cpp_helper_libs::quantities::Angle boundary_sum,
                   std::size_t boundary_segment_count,
                   std::vector<MinorArc> boundary_edges) noexcept
      : focus_one_(focus_one), focus_two_(focus_two), boundary_sum_(boundary_sum),
        boundary_segment_count_(boundary_segment_count),
        boundary_edges_(std::move(boundary_edges)) {}

  /**
   * @brief Shared containment evaluator for inclusive/exclusive and tolerant/exact variants.
   */
  bool contains_policy(const cpp_helper_libs::linear_algebra::UnitVector3 &point, bool inclusive,
                       bool exact) const noexcept;

  /**
   * @brief Shared boundary-intersection evaluator for inclusive/exclusive and tolerant/exact
   * variants.
   */
  bool boundary_intersects_policy(const SphericalCurve &curve, bool inclusive,
                                  bool exact) const noexcept;

  /// First focus of the spherical ellipse.
  cpp_helper_libs::linear_algebra::UnitVector3 focus_one_;
  /// Second focus of the spherical ellipse.
  cpp_helper_libs::linear_algebra::UnitVector3 focus_two_;
  /// Constant sum of central angles to the two foci on the boundary.
  cpp_helper_libs::quantities::Angle boundary_sum_;
  /// Number of segments used for the internal boundary approximation.
  std::size_t boundary_segment_count_;
  /// Piecewise-minor-arc approximation of the boundary used for intersection checks.
  std::vector<MinorArc> boundary_edges_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_ELLIPSE_HPP
