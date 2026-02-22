// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_MATRIX3_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_MATRIX3_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "cpp_helper_libs/linear_algebra/decompositions.hpp"
#include "cpp_helper_libs/linear_algebra/vector3.hpp"

namespace cpp_helper_libs::linear_algebra {

/**
 * @brief Immutable 3x3 matrix stored in row-major order.
 */
class Matrix3 final {
public:
  /**
   * @brief Construct a 3x3 matrix from row-major elements.
   */
  constexpr Matrix3(double row0_col0, double row0_col1, double row0_col2, double row1_col0,
                    double row1_col1, double row1_col2, double row2_col0, double row2_col1,
                    double row2_col2) noexcept
      : values_{row0_col0, row0_col1, row0_col2, row1_col0, row1_col1,
                row1_col2, row2_col0, row2_col1, row2_col2} {}

  /**
   * @brief Construct an identity matrix.
   */
  static Matrix3 identity() noexcept;

  /**
   * @brief Construct a matrix of all zeros.
   */
  static Matrix3 zeros() noexcept;

  /**
   * @brief Construct a matrix of all ones.
   */
  static Matrix3 ones() noexcept;

  /**
   * @brief Construct a pseudo-random matrix with deterministic seed.
   *
   * @param seed Random seed.
   * @param min_value Minimum generated value.
   * @param max_value Maximum generated value.
   * @return Randomly initialized matrix.
   */
  static Matrix3 random(std::uint64_t seed, double min_value = 0.0,
                        double max_value = 1.0) noexcept;

  /**
   * @brief Access matrix element at row/column.
   *
   * @param row_index Row index in `[0, 2]`.
   * @param column_index Column index in `[0, 2]`.
   * @return Element value at `(row_index, column_index)`.
   * @pre `row_index < 3 && column_index < 3`.
   */
  double operator()(std::size_t row_index, std::size_t column_index) const noexcept;

  /**
   * @brief Exact element-wise equality.
   */
  bool operator==(const Matrix3 &other) const noexcept;

  /**
   * @brief Exact element-wise inequality.
   */
  bool operator!=(const Matrix3 &other) const noexcept;

  /**
   * @brief Element-wise matrix addition.
   */
  Matrix3 operator+(const Matrix3 &other) const noexcept;

  /**
   * @brief Element-wise matrix subtraction.
   */
  Matrix3 operator-(const Matrix3 &other) const noexcept;

  /**
   * @brief Unary negation.
   */
  Matrix3 operator-() const noexcept;

  /**
   * @brief Scale all elements by a scalar.
   */
  Matrix3 operator*(double scalar) const noexcept;

  /**
   * @brief Scale all elements by a scalar.
   */
  friend Matrix3 operator*(double scalar, const Matrix3 &matrix) noexcept;

  /**
   * @brief Element-wise Hadamard product.
   */
  Matrix3 hadamard_product(const Matrix3 &other) const noexcept;

  /**
   * @brief Element-wise Hadamard division.
   *
   * @return Divided matrix, or `std::nullopt` if any divisor element is zero.
   */
  std::optional<Matrix3> hadamard_divide(const Matrix3 &other) const noexcept;

  /**
   * @brief Frobenius inner product with another matrix.
   */
  double dot(const Matrix3 &other) const noexcept;

  /**
   * @brief Matrix-vector multiplication.
   */
  Vector3 multiply(const Vector3 &vector) const noexcept;

  /**
   * @brief Matrix-matrix multiplication.
   */
  Matrix3 multiply(const Matrix3 &other) const noexcept;

  /**
   * @brief Entry-wise L1 norm.
   */
  double l1_norm() const noexcept;

  /**
   * @brief Entry-wise L2 (Frobenius) norm.
   */
  double l2_norm() const noexcept;

  /**
   * @brief Entry-wise max norm.
   */
  double max_norm() const noexcept;

  /**
   * @brief Matrix transpose.
   */
  Matrix3 transpose() const noexcept;

  /**
   * @brief Matrix trace.
   */
  double trace() const noexcept;

  /**
   * @brief Matrix determinant.
   */
  double determinant() const noexcept;

  /**
   * @brief Invert matrix if nonsingular.
   *
   * @return Inverse matrix, or `std::nullopt` if singular.
   */
  std::optional<Matrix3> inverse() const noexcept;

  /**
   * @brief Solve linear system `A x = b`.
   *
   * @param right_hand_side Vector `b`.
   * @param method Solver strategy.
   * @return Solution vector `x`, or `std::nullopt` if solving fails.
   */
  std::optional<Vector3> solve(const Vector3 &right_hand_side,
                               SolveMethod method = SolveMethod::Auto) const noexcept;

  /**
   * @brief Compute LU decomposition with partial pivoting.
   */
  std::optional<LU3> lu() const noexcept;

  /**
   * @brief Compute QR decomposition.
   */
  std::optional<QR3> qr() const noexcept;

  /**
   * @brief Compute Cholesky factorization.
   */
  std::optional<Cholesky3> cholesky() const noexcept;

private:
  explicit constexpr Matrix3(const std::array<double, 9> &values) noexcept : values_(values) {}

  /// Flat row-major storage of the 3x3 entries.
  std::array<double, 9> values_;
};

} // namespace cpp_helper_libs::linear_algebra

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_MATRIX3_HPP
