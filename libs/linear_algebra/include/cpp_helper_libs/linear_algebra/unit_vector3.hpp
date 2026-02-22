#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_UNIT_VECTOR3_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_UNIT_VECTOR3_HPP

#include <optional>

#include "cpp_helper_libs/quantities/angle.hpp"

namespace cpp_helper_libs::linear_algebra {

class Vector3;

class UnitVector3 final {
public:
  static std::optional<UnitVector3> from_components(double x_component, double y_component,
                                                    double z_component) noexcept;
  static std::optional<UnitVector3> from_vector(const Vector3 &value) noexcept;

  double x() const noexcept { return x_; }
  double y() const noexcept { return y_; }
  double z() const noexcept { return z_; }

  bool operator==(const UnitVector3 &other) const noexcept;
  bool operator!=(const UnitVector3 &other) const noexcept;
  UnitVector3 operator-() const noexcept;

  Vector3 as_vector() const noexcept;
  Vector3 scaled_by(double scalar) const noexcept;
  double dot(const UnitVector3 &other) const noexcept;
  double dot(const Vector3 &other) const noexcept;
  Vector3 cross(const UnitVector3 &other) const noexcept;
  Vector3 cross(const Vector3 &other) const noexcept;

  double central_angle_radians(const UnitVector3 &other) const noexcept;
  cpp_helper_libs::quantities::Angle central_angle(const UnitVector3 &other) const noexcept;

private:
  constexpr UnitVector3(double x, double y, double z) noexcept : x_(x), y_(y), z_(z) {}

  const double x_;
  const double y_;
  const double z_;
};

} // namespace cpp_helper_libs::linear_algebra

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_UNIT_VECTOR3_HPP
