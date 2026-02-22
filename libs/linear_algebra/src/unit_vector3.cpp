#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"

#include <algorithm>
#include <cmath>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"

namespace cpp_helper_libs::linear_algebra {
namespace {

// Unit-magnitude check tolerance for floating-point roundoff.
constexpr double kUnitLength = 1.0;
constexpr double kUnitLengthTolerance = 1e-12;

} // namespace

std::optional<UnitVector3> UnitVector3::from_components(const double x_component,
                                                        const double y_component,
                                                        const double z_component) noexcept {
  const double magnitude = std::sqrt((x_component * x_component) + (y_component * y_component) +
                                     (z_component * z_component));

  // Validate a strict unit-vector invariant while allowing tiny FP noise.
  if (std::fabs(magnitude - kUnitLength) > kUnitLengthTolerance) {
    return std::nullopt;
  }

  // NOLINTNEXTLINE(modernize-return-braced-init-list): optional cannot use private ctor.
  return UnitVector3(x_component, y_component, z_component);
}

std::optional<UnitVector3> UnitVector3::from_vector(const Vector3 &value) noexcept {
  const double magnitude = value.magnitude();

  // Degenerate vectors cannot be normalized into a unit vector.
  if (magnitude == 0.0) {
    return std::nullopt;
  }

  // NOLINTNEXTLINE(modernize-return-braced-init-list): optional cannot use private ctor.
  return UnitVector3(value.x() / magnitude, value.y() / magnitude, value.z() / magnitude);
}

bool UnitVector3::operator==(const UnitVector3 &other) const noexcept {
  return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
}

bool UnitVector3::operator!=(const UnitVector3 &other) const noexcept { return !(*this == other); }

UnitVector3 UnitVector3::operator-() const noexcept { return {-x_, -y_, -z_}; }

Vector3 UnitVector3::as_vector() const noexcept { return {x_, y_, z_}; }

Vector3 UnitVector3::scaled_by(const double scalar) const noexcept { return as_vector() * scalar; }

double UnitVector3::dot(const UnitVector3 &other) const noexcept {
  return (x_ * other.x_) + (y_ * other.y_) + (z_ * other.z_);
}

double UnitVector3::dot(const Vector3 &other) const noexcept {
  return (x_ * other.x()) + (y_ * other.y()) + (z_ * other.z());
}

Vector3 UnitVector3::cross(const UnitVector3 &other) const noexcept {
  return {(y_ * other.z_) - (z_ * other.y_), (z_ * other.x_) - (x_ * other.z_),
          (x_ * other.y_) - (y_ * other.x_)};
}

Vector3 UnitVector3::cross(const Vector3 &other) const noexcept {
  return {(y_ * other.z()) - (z_ * other.y()), (z_ * other.x()) - (x_ * other.z()),
          (x_ * other.y()) - (y_ * other.x())};
}

double UnitVector3::central_angle_radians(const UnitVector3 &other) const noexcept {
  const double cosine_value = dot(other);
  // Clamp small floating-point drift before calling acos.
  const double clamped_cosine = std::clamp(cosine_value, -1.0, 1.0);

  return std::acos(clamped_cosine);
}

cpp_helper_libs::quantities::Angle
UnitVector3::central_angle(const UnitVector3 &other) const noexcept {
  // Keep Angle-returning API behavior consistent with radians-returning API.
  return cpp_helper_libs::quantities::Angle::radians(central_angle_radians(other));
}

} // namespace cpp_helper_libs::linear_algebra
