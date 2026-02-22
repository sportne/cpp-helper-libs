// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_UNIT_VECTOR3_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_UNIT_VECTOR3_HPP

#include <optional>

#include "cpp_helper_libs/quantities/angle.hpp"

namespace cpp_helper_libs::linear_algebra {

class Vector3;

/**
 * @brief Immutable 3D unit vector.
 *
 * Instances are created through checked factories that enforce near-unit magnitude.
 */
class UnitVector3 final {
public:
  /**
   * @brief Create from explicit components when they represent unit length.
   *
   * @param x_component X component.
   * @param y_component Y component.
   * @param z_component Z component.
   * @return Unit vector if components satisfy unit-length tolerance; otherwise `std::nullopt`.
   */
  static std::optional<UnitVector3> from_components(double x_component, double y_component,
                                                    double z_component) noexcept;

  /**
   * @brief Create by normalizing a non-zero vector.
   *
   * @param value Source vector.
   * @return Normalized unit vector, or `std::nullopt` when @p value is zero.
   */
  static std::optional<UnitVector3> from_vector(const Vector3 &value) noexcept;

  /**
   * @brief Get the x component.
   *
   * @return X component value.
   */
  double x() const noexcept { return x_; }

  /**
   * @brief Get the y component.
   *
   * @return Y component value.
   */
  double y() const noexcept { return y_; }

  /**
   * @brief Get the z component.
   *
   * @return Z component value.
   */
  double z() const noexcept { return z_; }

  /**
   * @brief Compare two unit vectors for exact component-wise equality.
   *
   * @param other Unit vector to compare.
   * @return `true` when all components are exactly equal.
   */
  bool operator==(const UnitVector3 &other) const noexcept;

  /**
   * @brief Compare two unit vectors for exact component-wise inequality.
   *
   * @param other Unit vector to compare.
   * @return `true` when any component differs.
   */
  bool operator!=(const UnitVector3 &other) const noexcept;

  /**
   * @brief Negate all components.
   *
   * @return Unit vector in the opposite direction.
   */
  UnitVector3 operator-() const noexcept;

  /**
   * @brief Convert to `Vector3` with identical components.
   *
   * @return Equivalent non-unit vector value.
   */
  Vector3 as_vector() const noexcept;

  /**
   * @brief Scale by a scalar and return a non-unit vector.
   *
   * @param scalar Scalar factor.
   * @return Scaled vector.
   */
  Vector3 scaled_by(double scalar) const noexcept;

  /**
   * @brief Compute dot product with another unit vector.
   *
   * @param other Unit vector operand.
   * @return Dot product scalar.
   */
  double dot(const UnitVector3 &other) const noexcept;

  /**
   * @brief Compute dot product with a vector.
   *
   * @param other Vector operand.
   * @return Dot product scalar.
   */
  double dot(const Vector3 &other) const noexcept;

  /**
   * @brief Compute right-handed cross product with another unit vector.
   *
   * @param other Unit vector operand.
   * @return Cross-product vector.
   */
  Vector3 cross(const UnitVector3 &other) const noexcept;

  /**
   * @brief Compute right-handed cross product with a vector.
   *
   * @param other Vector operand.
   * @return Cross-product vector.
   */
  Vector3 cross(const Vector3 &other) const noexcept;

  /**
   * @brief Compute central angle to another unit vector in radians.
   *
   * @param other Other unit vector.
   * @return Angle between unit vectors in radians.
   */
  double central_angle_radians(const UnitVector3 &other) const noexcept;

  /**
   * @brief Compute central angle to another unit vector as `quantities::Angle`.
   *
   * @param other Other unit vector.
   * @return Angle between unit vectors.
   */
  cpp_helper_libs::quantities::Angle central_angle(const UnitVector3 &other) const noexcept;

private:
  /**
   * @brief Private constructor enforcing factory-based invariant checking.
   */
  constexpr UnitVector3(double x, double y, double z) noexcept : x_(x), y_(y), z_(z) {}

  const double x_;
  const double y_;
  const double z_;
};

} // namespace cpp_helper_libs::linear_algebra

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_UNIT_VECTOR3_HPP
