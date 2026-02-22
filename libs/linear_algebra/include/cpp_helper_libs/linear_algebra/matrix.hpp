// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_MATRIX_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_MATRIX_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace cpp_helper_libs::linear_algebra {

/**
 * @brief Immutable dynamic matrix stored in row-major order.
 */
class Matrix final {
public:
  /**
   * @brief Construct matrix from row-major values.
   *
   * @param row_count Number of rows.
   * @param column_count Number of columns.
   * @param values Row-major values.
   * @return Matrix value when dimensions match values size; otherwise `std::nullopt`.
   */
  static std::optional<Matrix> from_row_major(std::size_t row_count, std::size_t column_count,
                                              std::vector<double> values) noexcept;

  /**
   * @brief Construct zero matrix.
   */
  static Matrix zeros(std::size_t row_count, std::size_t column_count) noexcept;

  /**
   * @brief Construct ones matrix.
   */
  static Matrix ones(std::size_t row_count, std::size_t column_count) noexcept;

  /**
   * @brief Construct square identity matrix.
   */
  static Matrix identity(std::size_t dimension) noexcept;

  /**
   * @brief Construct random matrix with deterministic seed.
   */
  static Matrix random(std::size_t row_count, std::size_t column_count, std::uint64_t seed,
                       double min_value = 0.0, double max_value = 1.0) noexcept;

  /**
   * @brief Number of rows.
   */
  std::size_t rows() const noexcept { return row_count_; }

  /**
   * @brief Number of columns.
   */
  std::size_t cols() const noexcept { return column_count_; }

  /**
   * @brief Read matrix element at row/column.
   *
   * @pre `row_index < rows()` and `column_index < cols()`.
   */
  double operator()(std::size_t row_index, std::size_t column_index) const noexcept;

  /**
   * @brief Exact element-wise equality.
   */
  bool operator==(const Matrix &other) const noexcept;

  /**
   * @brief Exact element-wise inequality.
   */
  bool operator!=(const Matrix &other) const noexcept;

  /**
   * @brief Element-wise addition.
   *
   * @return Sum matrix or `std::nullopt` on shape mismatch.
   */
  std::optional<Matrix> add(const Matrix &other) const noexcept;

  /**
   * @brief Element-wise subtraction.
   *
   * @return Difference matrix or `std::nullopt` on shape mismatch.
   */
  std::optional<Matrix> subtract(const Matrix &other) const noexcept;

  /**
   * @brief Element-wise Hadamard product.
   *
   * @return Product matrix or `std::nullopt` on shape mismatch.
   */
  std::optional<Matrix> hadamard_product(const Matrix &other) const noexcept;

  /**
   * @brief Element-wise Hadamard division.
   *
   * @return Division matrix or `std::nullopt` on shape mismatch or zero divisor element.
   */
  std::optional<Matrix> hadamard_divide(const Matrix &other) const noexcept;

  /**
   * @brief Matrix multiplication.
   *
   * @return Product matrix or `std::nullopt` on shape mismatch.
   */
  std::optional<Matrix> multiply(const Matrix &other) const noexcept;

  /**
   * @brief Frobenius inner product.
   *
   * @return Scalar dot product or `std::nullopt` on shape mismatch.
   */
  std::optional<double> dot(const Matrix &other) const noexcept;

  /**
   * @brief Matrix transpose.
   */
  Matrix transpose() const noexcept;

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

private:
  Matrix(std::size_t row_count, std::size_t column_count, std::vector<double> values) noexcept
      : row_count_(row_count), column_count_(column_count), values_(std::move(values)) {}

  std::size_t row_count_;
  std::size_t column_count_;
  std::vector<double> values_;
};

} // namespace cpp_helper_libs::linear_algebra

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_MATRIX_HPP
