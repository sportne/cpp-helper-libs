// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_VECTOR3_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_VECTOR3_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"

namespace cpp_helper_libs::linear_algebra {

/**
 * @brief Immutable 3D cartesian vector.
 *
 * Equality is exact and component-wise.
 */
class Vector3 final {
public:
  /**
   * @brief Construct a vector from cartesian components.
   *
   * @param x X component.
   * @param y Y component.
   * @param z Z component.
   */
  constexpr Vector3(double x, double y, double z) noexcept : x_(x), y_(y), z_(z) {}

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
   * @brief Compare two vectors for exact component-wise equality.
   *
   * @param other Vector to compare.
   * @return `true` if all components are exactly equal.
   */
  bool operator==(const Vector3 &other) const noexcept;

  /**
   * @brief Compare two vectors for exact component-wise inequality.
   *
   * @param other Vector to compare.
   * @return `true` if any component differs.
   */
  bool operator!=(const Vector3 &other) const noexcept;

  /**
   * @brief Add vectors component-wise.
   *
   * @param other Vector addend.
   * @return Sum vector.
   */
  Vector3 operator+(const Vector3 &other) const noexcept;

  /**
   * @brief Subtract vectors component-wise.
   *
   * @param other Vector subtrahend.
   * @return Difference vector.
   */
  Vector3 operator-(const Vector3 &other) const noexcept;

  /**
   * @brief Negate each component.
   *
   * @return Negated vector.
   */
  Vector3 operator-() const noexcept;

  /**
   * @brief Scale by a scalar factor.
   *
   * @param scalar Scalar factor.
   * @return Scaled vector.
   */
  Vector3 operator*(double scalar) const noexcept;

  /**
   * @brief Scale by a scalar factor.
   *
   * @param scalar Scalar factor.
   * @param vector Vector value.
   * @return Scaled vector.
   */
  friend Vector3 operator*(double scalar, const Vector3 &vector) noexcept;

  /**
   * @brief Compute dot product.
   *
   * @param other Vector operand.
   * @return Dot product scalar.
   */
  double dot(const Vector3 &other) const noexcept;

  /**
   * @brief Compute right-handed cross product.
   *
   * @param other Vector operand.
   * @return Cross-product vector.
   */
  Vector3 cross(const Vector3 &other) const noexcept;

  /**
   * @brief Compute squared magnitude.
   *
   * @return `x^2 + y^2 + z^2`.
   */
  double squared_magnitude() const noexcept;

  /**
   * @brief Compute Euclidean magnitude.
   *
   * @return `sqrt(x^2 + y^2 + z^2)`.
   */
  double magnitude() const noexcept;

  /**
   * @brief Compute Euclidean distance to another vector.
   *
   * @param other Target vector.
   * @return Euclidean distance between the vectors.
   */
  double euclidean_distance_to(const Vector3 &other) const noexcept;

  /**
   * @brief Normalize this vector to unit length.
   *
   * @return Unit vector in the same direction, or `std::nullopt` for zero vector input.
   */
  std::optional<UnitVector3> normalized() const noexcept;

  /**
   * @brief Compute central angle to another vector in radians.
   *
   * @param other Other vector.
   * @return Angle in radians, or `std::nullopt` if either vector has zero magnitude.
   */
  std::optional<double> central_angle_radians(const Vector3 &other) const noexcept;

  /**
   * @brief Compute central angle to another vector as a quantity type.
   *
   * @param other Other vector.
   * @return Angle quantity, or `std::nullopt` if either vector has zero magnitude.
   */
  std::optional<cpp_helper_libs::quantities::Angle>
  central_angle(const Vector3 &other) const noexcept;

private:
  const double x_;
  const double y_;
  const double z_;
};

/**
 * @brief Compute the unit tangent from a direction vector.
 *
 * @param direction Direction vector.
 * @return `normalize(direction)`, or `std::nullopt` for zero vector input.
 */
std::optional<UnitVector3> unit_tangent(const Vector3 &direction) noexcept;

/**
 * @brief Compute a unit normal from two vectors.
 *
 * @param first First input vector.
 * @param second Second input vector.
 * @return `normalize(cross(first, second))`, or `std::nullopt` when cross product is zero.
 */
std::optional<UnitVector3> unit_normal(const Vector3 &first, const Vector3 &second) noexcept;

} // namespace cpp_helper_libs::linear_algebra

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_VECTOR3_HPP
