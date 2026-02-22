#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_VECTOR3_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_VECTOR3_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"

namespace cpp_helper_libs::linear_algebra {

class Vector3 final {
public:
  constexpr Vector3(double x, double y, double z) noexcept : x_(x), y_(y), z_(z) {}

  double x() const noexcept { return x_; }
  double y() const noexcept { return y_; }
  double z() const noexcept { return z_; }

  bool operator==(const Vector3 &other) const noexcept;
  bool operator!=(const Vector3 &other) const noexcept;

  Vector3 operator+(const Vector3 &other) const noexcept;
  Vector3 operator-(const Vector3 &other) const noexcept;
  Vector3 operator-() const noexcept;
  Vector3 operator*(double scalar) const noexcept;
  friend Vector3 operator*(double scalar, const Vector3 &vector) noexcept;

  double dot(const Vector3 &other) const noexcept;
  Vector3 cross(const Vector3 &other) const noexcept;

  double squared_magnitude() const noexcept;
  double magnitude() const noexcept;
  double euclidean_distance_to(const Vector3 &other) const noexcept;

  std::optional<UnitVector3> normalized() const noexcept;
  std::optional<double> central_angle_radians(const Vector3 &other) const noexcept;
  std::optional<cpp_helper_libs::quantities::Angle> central_angle(const Vector3 &other) const
      noexcept;

private:
  const double x_;
  const double y_;
  const double z_;
};

std::optional<UnitVector3> unit_tangent(const Vector3 &direction) noexcept;
std::optional<UnitVector3> unit_normal(const Vector3 &first, const Vector3 &second) noexcept;

} // namespace cpp_helper_libs::linear_algebra

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_VECTOR3_HPP
