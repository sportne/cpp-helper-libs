// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT
#include "cpp_helper_libs/linear_algebra/matrix.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

// NOLINTBEGIN(bugprone-exception-escape,modernize-return-braced-init-list,bugprone-easily-swappable-parameters,readability-suspicious-call-argument)
namespace cpp_helper_libs::linear_algebra {
namespace {

std::optional<std::size_t> product_if_safe(const std::size_t left_value,
                                           const std::size_t right_value) noexcept {
  // Safe multiplication helper for dimensions.
  // Returns nullopt if rows*cols would overflow size_t, which prevents
  // allocating a vector with a wrapped (incorrect) size.
  if (left_value == 0U || right_value == 0U) {
    return 0U;
  }
  if (left_value > (std::numeric_limits<std::size_t>::max() / right_value)) {
    return std::nullopt;
  }
  return left_value * right_value;
}

std::size_t linear_index(const std::size_t row_index, const std::size_t column_index,
                         const std::size_t column_count) noexcept {
  // Row-major layout mapping:
  // [r0c0 r0c1 ... r1c0 r1c1 ...]
  return (row_index * column_count) + column_index;
}

} // namespace

std::optional<Matrix> Matrix::from_row_major(const std::size_t row_count,
                                             const std::size_t column_count,
                                             std::vector<double> values) noexcept {
  // Construction succeeds only when provided values exactly match rows*cols.
  // This keeps Matrix immutable and always internally consistent.
  const std::optional<std::size_t> expected_size = product_if_safe(row_count, column_count);
  if (!expected_size.has_value() || values.size() != expected_size.value()) {
    return std::nullopt;
  }

  return Matrix(row_count, column_count, std::move(values));
}

Matrix Matrix::zeros(const std::size_t row_count, const std::size_t column_count) noexcept {
  const std::optional<std::size_t> element_count = product_if_safe(row_count, column_count);
  if (!element_count.has_value()) {
    return Matrix(0U, 0U, {});
  }

  return Matrix(row_count, column_count, std::vector<double>(element_count.value(), 0.0));
}

Matrix Matrix::ones(const std::size_t row_count, const std::size_t column_count) noexcept {
  const std::optional<std::size_t> element_count = product_if_safe(row_count, column_count);
  if (!element_count.has_value()) {
    return Matrix(0U, 0U, {});
  }

  return Matrix(row_count, column_count, std::vector<double>(element_count.value(), 1.0));
}

Matrix Matrix::identity(const std::size_t dimension) noexcept {
  Matrix result = zeros(dimension, dimension);
  // Identity matrix has 1s on the diagonal and 0s elsewhere.
  // It is the multiplicative neutral element for square matrices.
  for (std::size_t diagonal_index = 0U; diagonal_index < dimension; ++diagonal_index) {
    result.values_[linear_index(diagonal_index, diagonal_index, dimension)] = 1.0;
  }
  return result;
}

Matrix Matrix::random(const std::size_t row_count, const std::size_t column_count,
                      const std::uint64_t seed, const double min_value,
                      const double max_value) noexcept {
  const std::optional<std::size_t> element_count = product_if_safe(row_count, column_count);
  if (!element_count.has_value()) {
    return Matrix(0U, 0U, {});
  }

  std::vector<double> values(element_count.value(), 0.0);
  std::mt19937_64 random_engine(seed);
  std::uniform_real_distribution<double> random_distribution(min_value, max_value);

  // Deterministic random fill (same seed => same matrix), useful for tests.
  std::generate(values.begin(), values.end(), [&random_distribution, &random_engine]() {
    return random_distribution(random_engine);
  });

  return Matrix(row_count, column_count, std::move(values));
}

double Matrix::operator()(const std::size_t row_index,
                          const std::size_t column_index) const noexcept {
  return values_[linear_index(row_index, column_index, column_count_)];
}

bool Matrix::operator==(const Matrix &other) const noexcept {
  return row_count_ == other.row_count_ && column_count_ == other.column_count_ &&
         values_ == other.values_;
}

bool Matrix::operator!=(const Matrix &other) const noexcept { return !(*this == other); }

std::optional<Matrix> Matrix::add(const Matrix &other) const noexcept {
  // Element-wise operations require identical shape.
  if (row_count_ != other.row_count_ || column_count_ != other.column_count_) {
    return std::nullopt;
  }

  std::vector<double> result_values(values_.size(), 0.0);
  for (std::size_t entry_index = 0U; entry_index < values_.size(); ++entry_index) {
    result_values[entry_index] = values_[entry_index] + other.values_[entry_index];
  }

  return Matrix(row_count_, column_count_, std::move(result_values));
}

std::optional<Matrix> Matrix::subtract(const Matrix &other) const noexcept {
  if (row_count_ != other.row_count_ || column_count_ != other.column_count_) {
    return std::nullopt;
  }

  std::vector<double> result_values(values_.size(), 0.0);
  for (std::size_t entry_index = 0U; entry_index < values_.size(); ++entry_index) {
    result_values[entry_index] = values_[entry_index] - other.values_[entry_index];
  }

  return Matrix(row_count_, column_count_, std::move(result_values));
}

std::optional<Matrix> Matrix::hadamard_product(const Matrix &other) const noexcept {
  // Hadamard product is entry-by-entry multiplication, not matrix multiplication.
  if (row_count_ != other.row_count_ || column_count_ != other.column_count_) {
    return std::nullopt;
  }

  std::vector<double> result_values(values_.size(), 0.0);
  for (std::size_t entry_index = 0U; entry_index < values_.size(); ++entry_index) {
    result_values[entry_index] = values_[entry_index] * other.values_[entry_index];
  }

  return Matrix(row_count_, column_count_, std::move(result_values));
}

std::optional<Matrix> Matrix::hadamard_divide(const Matrix &other) const noexcept {
  if (row_count_ != other.row_count_ || column_count_ != other.column_count_) {
    return std::nullopt;
  }

  std::vector<double> result_values(values_.size(), 0.0);
  for (std::size_t entry_index = 0U; entry_index < values_.size(); ++entry_index) {
    // Division by zero at any entry makes the full operation undefined.
    if (other.values_[entry_index] == 0.0) {
      return std::nullopt;
    }
    result_values[entry_index] = values_[entry_index] / other.values_[entry_index];
  }

  return Matrix(row_count_, column_count_, std::move(result_values));
}

std::optional<Matrix> Matrix::multiply(const Matrix &other) const noexcept {
  // Matrix multiplication shape rule:
  // (m x n) * (n x p) -> (m x p)
  if (column_count_ != other.row_count_) {
    return std::nullopt;
  }

  const std::optional<std::size_t> product_size = product_if_safe(row_count_, other.column_count_);
  if (!product_size.has_value()) {
    return std::nullopt;
  }

  std::vector<double> result_values(product_size.value(), 0.0);

  // Standard row-by-column dot-product multiplication.
  for (std::size_t row_index = 0U; row_index < row_count_; ++row_index) {
    for (std::size_t column_index = 0U; column_index < other.column_count_; ++column_index) {
      double sum_value = 0.0;
      for (std::size_t shared_index = 0U; shared_index < column_count_; ++shared_index) {
        sum_value += (*this)(row_index, shared_index) * other(shared_index, column_index);
      }
      result_values[linear_index(row_index, column_index, other.column_count_)] = sum_value;
    }
  }

  return Matrix(row_count_, other.column_count_, std::move(result_values));
}

std::optional<double> Matrix::dot(const Matrix &other) const noexcept {
  // Matrix dot product here means Frobenius inner product:
  // sum over all entries A(i,j) * B(i,j).
  if (row_count_ != other.row_count_ || column_count_ != other.column_count_) {
    return std::nullopt;
  }

  double sum_value = 0.0;
  for (std::size_t entry_index = 0U; entry_index < values_.size(); ++entry_index) {
    sum_value += values_[entry_index] * other.values_[entry_index];
  }

  return sum_value;
}

Matrix Matrix::transpose() const noexcept {
  // Transpose flips rows and columns:
  // result(j, i) = original(i, j)
  const std::optional<std::size_t> element_count = product_if_safe(row_count_, column_count_);
  if (!element_count.has_value()) {
    return Matrix(0U, 0U, {});
  }

  std::vector<double> result_values(element_count.value(), 0.0);
  for (std::size_t row_index = 0U; row_index < row_count_; ++row_index) {
    for (std::size_t column_index = 0U; column_index < column_count_; ++column_index) {
      result_values[linear_index(column_index, row_index, row_count_)] =
          (*this)(row_index, column_index);
    }
  }

  return Matrix(column_count_, row_count_, std::move(result_values));
}

double Matrix::l1_norm() const noexcept {
  // Entry-wise L1 norm: sum(abs(a_ij)).
  return std::accumulate(
      values_.begin(), values_.end(), 0.0,
      [](const double accumulated, const double entry) { return accumulated + std::fabs(entry); });
}

double Matrix::l2_norm() const noexcept {
  // Entry-wise L2 norm (Frobenius norm): sqrt(sum(a_ij^2)).
  const double squared_sum = std::accumulate(
      values_.begin(), values_.end(), 0.0,
      [](const double accumulated, const double entry) { return accumulated + (entry * entry); });
  return std::sqrt(squared_sum);
}

double Matrix::max_norm() const noexcept {
  // Entry-wise max norm: max(abs(a_ij)).
  return std::accumulate(values_.begin(), values_.end(), 0.0,
                         [](const double max_value, const double entry) {
                           return std::max(max_value, std::fabs(entry));
                         });
}

} // namespace cpp_helper_libs::linear_algebra
// NOLINTEND(bugprone-exception-escape,modernize-return-braced-init-list,bugprone-easily-swappable-parameters,readability-suspicious-call-argument)
