#include "cpp_helper_libs/linear_algebra/vector3.hpp"

#include <algorithm>
#include <cmath>

#include "cpp_helper_libs/quantities/angle.hpp"

namespace cpp_helper_libs::linear_algebra {

bool Vector3::operator==(const Vector3 &other) const noexcept {
  return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
}

bool Vector3::operator!=(const Vector3 &other) const noexcept { return !(*this == other); }

Vector3 Vector3::operator+(const Vector3 &other) const noexcept {
  return {x_ + other.x_, y_ + other.y_, z_ + other.z_};
}

Vector3 Vector3::operator-(const Vector3 &other) const noexcept {
  return {x_ - other.x_, y_ - other.y_, z_ - other.z_};
}

Vector3 Vector3::operator-() const noexcept { return {-x_, -y_, -z_}; }

Vector3 Vector3::operator*(const double scalar) const noexcept {
  return {x_ * scalar, y_ * scalar, z_ * scalar};
}

Vector3 operator*(const double scalar, const Vector3 &vector) noexcept { return vector * scalar; }

double Vector3::dot(const Vector3 &other) const noexcept {
  return (x_ * other.x_) + (y_ * other.y_) + (z_ * other.z_);
}

Vector3 Vector3::cross(const Vector3 &other) const noexcept {
  return {(y_ * other.z_) - (z_ * other.y_), (z_ * other.x_) - (x_ * other.z_),
          (x_ * other.y_) - (y_ * other.x_)};
}

double Vector3::squared_magnitude() const noexcept { return dot(*this); }

double Vector3::magnitude() const noexcept { return std::sqrt(squared_magnitude()); }

double Vector3::euclidean_distance_to(const Vector3 &other) const noexcept {
  return (*this - other).magnitude();
}

std::optional<UnitVector3> Vector3::normalized() const noexcept {
  return UnitVector3::from_vector(*this);
}

std::optional<double> Vector3::central_angle_radians(const Vector3 &other) const noexcept {
  const double lhs_magnitude = magnitude();
  const double rhs_magnitude = other.magnitude();

  // Degenerate vectors cannot define a central angle.
  if (lhs_magnitude == 0.0 || rhs_magnitude == 0.0) {
    return std::nullopt;
  }

  const double cosine_value = dot(other) / (lhs_magnitude * rhs_magnitude);
  // Clamp small floating-point drift before calling acos.
  const double clamped_cosine = std::clamp(cosine_value, -1.0, 1.0);

  return std::acos(clamped_cosine);
}

std::optional<cpp_helper_libs::quantities::Angle>
Vector3::central_angle(const Vector3 &other) const noexcept {
  const std::optional<double> radians = central_angle_radians(other);
  if (!radians.has_value()) {
    return std::nullopt;
  }

  // Preserve the optional degenerate-state contract when lifting to Angle.
  return cpp_helper_libs::quantities::Angle::radians(radians.value());
}

std::optional<UnitVector3> unit_tangent(const Vector3 &direction) noexcept {
  // Tangent is the normalized direction vector.
  return direction.normalized();
}

std::optional<UnitVector3> unit_normal(const Vector3 &first, const Vector3 &second) noexcept {
  // Normal is perpendicular to the input plane via right-handed cross product.
  return first.cross(second).normalized();
}

} // namespace cpp_helper_libs::linear_algebra
