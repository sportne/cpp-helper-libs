// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT
#include "cpp_helper_libs/linear_algebra/matrix3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <utility>

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,bugprone-easily-swappable-parameters,readability-suspicious-call-argument)
namespace cpp_helper_libs::linear_algebra {
namespace {

constexpr std::size_t kDimension = 3U;
constexpr std::size_t kMatrixEntryCount = kDimension * kDimension;
// Single tolerance used to decide when a floating-point value is "effectively zero".
// We use this for singularity checks, pivot checks, and Cholesky SPD validation.
constexpr double kNearZeroTolerance = 1e-12;

// Converts a (row, column) location into a flat row-major index:
//   [r0c0 r0c1 r0c2 r1c0 ...]
constexpr std::size_t linear_index(const std::size_t row_index,
                                   const std::size_t column_index) noexcept {
  return (row_index * kDimension) + column_index;
}

// Internal conversion helper to move between Vector3 and fixed-size arrays used by
// decomposition/solve helpers.
std::array<double, kDimension> vector_to_array(const Vector3 &vector_value) noexcept {
  return {vector_value.x(), vector_value.y(), vector_value.z()};
}

Vector3 array_to_vector(const std::array<double, kDimension> &values) noexcept {
  return {values[0], values[1], values[2]};
}

double dot3(const std::array<double, kDimension> &left_values,
            const std::array<double, kDimension> &right_values) noexcept {
  return (left_values[0] * right_values[0]) + (left_values[1] * right_values[1]) +
         (left_values[2] * right_values[2]);
}

double l2_norm3(const std::array<double, kDimension> &values) noexcept {
  return std::sqrt(dot3(values, values));
}

std::array<double, kDimension> scaled3(const std::array<double, kDimension> &values,
                                       const double scale_value) noexcept {
  return {values[0] * scale_value, values[1] * scale_value, values[2] * scale_value};
}

std::array<double, kDimension>
subtract3(const std::array<double, kDimension> &left_values,
          const std::array<double, kDimension> &right_values) noexcept {
  return {left_values[0] - right_values[0], left_values[1] - right_values[1],
          left_values[2] - right_values[2]};
}

std::optional<std::array<double, kDimension>>
forward_substitute(const std::array<double, kMatrixEntryCount> &lower_values,
                   const std::array<double, kDimension> &right_hand_side) noexcept {
  // Forward substitution solves Lx=b when L is lower triangular.
  // At row i, all x[0..i-1] are already known, so we can isolate x[i].
  //
  // Example row equation:
  //   L(i,0)x0 + ... + L(i,i-1)x(i-1) + L(i,i)xi = b(i)
  // Rearranged:
  //   xi = (b(i) - known_part) / L(i,i)
  std::array<double, kDimension> solution_values{0.0, 0.0, 0.0};

  for (std::size_t row_index = 0U; row_index < kDimension; ++row_index) {
    double sum_value = right_hand_side[row_index];
    for (std::size_t column_index = 0U; column_index < row_index; ++column_index) {
      sum_value -=
          lower_values[linear_index(row_index, column_index)] * solution_values[column_index];
    }

    const double diagonal_value = lower_values[linear_index(row_index, row_index)];
    // A near-zero diagonal means this triangular system is numerically unsolvable.
    if (std::fabs(diagonal_value) <= kNearZeroTolerance) {
      return std::nullopt;
    }
    solution_values[row_index] = sum_value / diagonal_value;
  }

  return solution_values;
}

std::optional<std::array<double, kDimension>>
back_substitute(const std::array<double, kMatrixEntryCount> &upper_values,
                const std::array<double, kDimension> &right_hand_side) noexcept {
  // Back substitution solves Ux=b when U is upper triangular.
  // We start at the last row because it has the fewest unknowns.
  // Then we move upward, plugging in values we already solved.
  std::array<double, kDimension> solution_values{0.0, 0.0, 0.0};

  for (std::size_t reverse_index = 0U; reverse_index < kDimension; ++reverse_index) {
    const std::size_t row_index = (kDimension - 1U) - reverse_index;

    double sum_value = right_hand_side[row_index];
    for (std::size_t column_index = row_index + 1U; column_index < kDimension; ++column_index) {
      sum_value -=
          upper_values[linear_index(row_index, column_index)] * solution_values[column_index];
    }

    const double diagonal_value = upper_values[linear_index(row_index, row_index)];
    // A near-zero diagonal indicates a singular/ill-conditioned step.
    if (std::fabs(diagonal_value) <= kNearZeroTolerance) {
      return std::nullopt;
    }
    solution_values[row_index] = sum_value / diagonal_value;
  }

  return solution_values;
}

std::optional<Vector3> solve_lu_system(const LU3 &decomposition,
                                       const Vector3 &right_hand_side) noexcept {
  // LU stores P*A = L*U (partial pivoting), so we solve:
  //   L*U*x = P*b
  // in two triangular stages:
  //   1) L*y = P*b
  //   2) U*x = y
  const std::array<double, kDimension> right_hand_side_values = vector_to_array(right_hand_side);

  // Apply permutation P to b using decomposition.permutation.
  std::array<double, kDimension> permuted_values{0.0, 0.0, 0.0};
  for (std::size_t row_index = 0U; row_index < kDimension; ++row_index) {
    permuted_values[row_index] = right_hand_side_values[decomposition.permutation[row_index]];
  }

  const std::optional<std::array<double, kDimension>> intermediate_values =
      forward_substitute(decomposition.lower, permuted_values);
  if (!intermediate_values.has_value()) {
    return std::nullopt;
  }

  const std::optional<std::array<double, kDimension>> solution_values =
      back_substitute(decomposition.upper, intermediate_values.value());
  if (!solution_values.has_value()) {
    return std::nullopt;
  }

  return array_to_vector(solution_values.value());
}

std::optional<Vector3> solve_qr_system(const QR3 &decomposition,
                                       const Vector3 &right_hand_side) noexcept {
  // QR stores A = Q*R with Q orthogonal (Q^-1 = Q^T).
  // Solve A*x=b by multiplying both sides by Q^T:
  //   Q^T*Q*R*x = Q^T*b  ->  R*x = Q^T*b
  // Then solve the upper-triangular system with back substitution.
  const std::array<double, kDimension> rhs_values = vector_to_array(right_hand_side);

  // Compute Q^T * b directly.
  std::array<double, kDimension> transposed_q_rhs{0.0, 0.0, 0.0};
  for (std::size_t column_index = 0U; column_index < kDimension; ++column_index) {
    for (std::size_t row_index = 0U; row_index < kDimension; ++row_index) {
      transposed_q_rhs[column_index] +=
          decomposition.orthogonal[linear_index(row_index, column_index)] * rhs_values[row_index];
    }
  }

  const std::optional<std::array<double, kDimension>> solution_values =
      back_substitute(decomposition.upper, transposed_q_rhs);
  if (!solution_values.has_value()) {
    return std::nullopt;
  }

  return array_to_vector(solution_values.value());
}

std::optional<Vector3> solve_cholesky_system(const Cholesky3 &decomposition,
                                             const Vector3 &right_hand_side) noexcept {
  // Cholesky stores A = L*L^T for symmetric positive-definite matrices.
  // Solve A*x=b in two triangular steps:
  //   1) L*y = b
  //   2) L^T*x = y
  const std::array<double, kDimension> rhs_values = vector_to_array(right_hand_side);

  const std::optional<std::array<double, kDimension>> intermediate_values =
      forward_substitute(decomposition.lower, rhs_values);
  if (!intermediate_values.has_value()) {
    return std::nullopt;
  }

  std::array<double, kMatrixEntryCount> transposed_lower{};
  // Materialize L^T (transpose of L) for the second substitution step.
  for (std::size_t row_index = 0U; row_index < kDimension; ++row_index) {
    for (std::size_t column_index = 0U; column_index < kDimension; ++column_index) {
      transposed_lower[linear_index(row_index, column_index)] =
          decomposition.lower[linear_index(column_index, row_index)];
    }
  }

  const std::optional<std::array<double, kDimension>> solution_values =
      back_substitute(transposed_lower, intermediate_values.value());
  if (!solution_values.has_value()) {
    return std::nullopt;
  }

  return array_to_vector(solution_values.value());
}

} // namespace

Matrix3 Matrix3::identity() noexcept { return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; }

Matrix3 Matrix3::zeros() noexcept { return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; }

Matrix3 Matrix3::ones() noexcept { return {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; }

Matrix3 Matrix3::random(const std::uint64_t seed, const double min_value,
                        const double max_value) noexcept {
  // Seeded RNG makes tests/repro cases deterministic.
  std::mt19937_64 random_engine(seed);
  std::uniform_real_distribution<double> random_distribution(min_value, max_value);

  return {random_distribution(random_engine), random_distribution(random_engine),
          random_distribution(random_engine), random_distribution(random_engine),
          random_distribution(random_engine), random_distribution(random_engine),
          random_distribution(random_engine), random_distribution(random_engine),
          random_distribution(random_engine)};
}

double Matrix3::operator()(const std::size_t row_index,
                           const std::size_t column_index) const noexcept {
  return values_[linear_index(row_index, column_index)];
}

bool Matrix3::operator==(const Matrix3 &other) const noexcept { return values_ == other.values_; }

bool Matrix3::operator!=(const Matrix3 &other) const noexcept { return !(*this == other); }

Matrix3 Matrix3::operator+(const Matrix3 &other) const noexcept {
  return {
      values_[0] + other.values_[0], values_[1] + other.values_[1], values_[2] + other.values_[2],
      values_[3] + other.values_[3], values_[4] + other.values_[4], values_[5] + other.values_[5],
      values_[6] + other.values_[6], values_[7] + other.values_[7], values_[8] + other.values_[8]};
}

Matrix3 Matrix3::operator-(const Matrix3 &other) const noexcept {
  return {
      values_[0] - other.values_[0], values_[1] - other.values_[1], values_[2] - other.values_[2],
      values_[3] - other.values_[3], values_[4] - other.values_[4], values_[5] - other.values_[5],
      values_[6] - other.values_[6], values_[7] - other.values_[7], values_[8] - other.values_[8]};
}

Matrix3 Matrix3::operator-() const noexcept {
  return {-values_[0], -values_[1], -values_[2], -values_[3], -values_[4],
          -values_[5], -values_[6], -values_[7], -values_[8]};
}

Matrix3 Matrix3::operator*(const double scalar) const noexcept {
  return {values_[0] * scalar, values_[1] * scalar, values_[2] * scalar,
          values_[3] * scalar, values_[4] * scalar, values_[5] * scalar,
          values_[6] * scalar, values_[7] * scalar, values_[8] * scalar};
}

Matrix3 operator*(const double scalar, const Matrix3 &matrix) noexcept { return matrix * scalar; }

Matrix3 Matrix3::hadamard_product(const Matrix3 &other) const noexcept {
  return {
      values_[0] * other.values_[0], values_[1] * other.values_[1], values_[2] * other.values_[2],
      values_[3] * other.values_[3], values_[4] * other.values_[4], values_[5] * other.values_[5],
      values_[6] * other.values_[6], values_[7] * other.values_[7], values_[8] * other.values_[8]};
}

std::optional<Matrix3> Matrix3::hadamard_divide(const Matrix3 &other) const noexcept {
  for (std::size_t entry_index = 0U; entry_index < values_.size(); ++entry_index) {
    if (other.values_[entry_index] == 0.0) {
      return std::nullopt;
    }
  }

  return Matrix3(
      values_[0] / other.values_[0], values_[1] / other.values_[1], values_[2] / other.values_[2],
      values_[3] / other.values_[3], values_[4] / other.values_[4], values_[5] / other.values_[5],
      values_[6] / other.values_[6], values_[7] / other.values_[7], values_[8] / other.values_[8]);
}

double Matrix3::dot(const Matrix3 &other) const noexcept {
  // Matrix dot product here is the Frobenius inner product:
  // sum over all entries of A(i,j) * B(i,j).
  double sum_value = 0.0;
  for (std::size_t entry_index = 0U; entry_index < values_.size(); ++entry_index) {
    sum_value += values_[entry_index] * other.values_[entry_index];
  }
  return sum_value;
}

Vector3 Matrix3::multiply(const Vector3 &vector) const noexcept {
  // Standard matrix-vector multiplication (row-by-column dot products).
  const double product_x =
      (values_[0] * vector.x()) + (values_[1] * vector.y()) + (values_[2] * vector.z());
  const double product_y =
      (values_[3] * vector.x()) + (values_[4] * vector.y()) + (values_[5] * vector.z());
  const double product_z =
      (values_[6] * vector.x()) + (values_[7] * vector.y()) + (values_[8] * vector.z());

  return {product_x, product_y, product_z};
}

Matrix3 Matrix3::multiply(const Matrix3 &other) const noexcept {
  // Standard matrix-matrix multiplication:
  // C(i,j) = sum_k A(i,k) * B(k,j).
  std::array<double, kMatrixEntryCount> result_values{};

  for (std::size_t row_index = 0U; row_index < kDimension; ++row_index) {
    for (std::size_t column_index = 0U; column_index < kDimension; ++column_index) {
      double sum_value = 0.0;
      for (std::size_t shared_index = 0U; shared_index < kDimension; ++shared_index) {
        sum_value += (*this)(row_index, shared_index) * other(shared_index, column_index);
      }
      result_values[linear_index(row_index, column_index)] = sum_value;
    }
  }

  return {result_values[0], result_values[1], result_values[2], result_values[3], result_values[4],
          result_values[5], result_values[6], result_values[7], result_values[8]};
}

double Matrix3::l1_norm() const noexcept {
  return std::accumulate(
      values_.begin(), values_.end(), 0.0,
      [](const double accumulated, const double entry) { return accumulated + std::fabs(entry); });
}

double Matrix3::l2_norm() const noexcept {
  const double squared_sum = std::accumulate(
      values_.begin(), values_.end(), 0.0,
      [](const double accumulated, const double entry) { return accumulated + (entry * entry); });
  return std::sqrt(squared_sum);
}

double Matrix3::max_norm() const noexcept {
  return std::accumulate(values_.begin(), values_.end(), 0.0,
                         [](const double max_value, const double entry) {
                           return std::max(max_value, std::fabs(entry));
                         });
}

Matrix3 Matrix3::transpose() const noexcept {
  return {values_[0], values_[3], values_[6], values_[1], values_[4],
          values_[7], values_[2], values_[5], values_[8]};
}

double Matrix3::trace() const noexcept { return values_[0] + values_[4] + values_[8]; }

double Matrix3::determinant() const noexcept {
  // Cofactor expansion along the first row for a 3x3 matrix.
  // Determinant describes signed volume scaling of the linear transform.
  // If det is near zero, the matrix collapses space and is not invertible.
  const double determinant_value =
      (values_[0] * ((values_[4] * values_[8]) - (values_[5] * values_[7]))) -
      (values_[1] * ((values_[3] * values_[8]) - (values_[5] * values_[6]))) +
      (values_[2] * ((values_[3] * values_[7]) - (values_[4] * values_[6])));

  return determinant_value;
}

std::optional<Matrix3> Matrix3::inverse() const noexcept {
  const double determinant_value = determinant();
  // Non-invertible (or numerically too close to singular) matrices return nullopt.
  if (std::fabs(determinant_value) <= kNearZeroTolerance) {
    return std::nullopt;
  }

  // Build the cofactor matrix entry-by-entry.
  const double cofactor00 = (values_[4] * values_[8]) - (values_[5] * values_[7]);
  const double cofactor01 = -((values_[3] * values_[8]) - (values_[5] * values_[6]));
  const double cofactor02 = (values_[3] * values_[7]) - (values_[4] * values_[6]);

  const double cofactor10 = -((values_[1] * values_[8]) - (values_[2] * values_[7]));
  const double cofactor11 = (values_[0] * values_[8]) - (values_[2] * values_[6]);
  const double cofactor12 = -((values_[0] * values_[7]) - (values_[1] * values_[6]));

  const double cofactor20 = (values_[1] * values_[5]) - (values_[2] * values_[4]);
  const double cofactor21 = -((values_[0] * values_[5]) - (values_[2] * values_[3]));
  const double cofactor22 = (values_[0] * values_[4]) - (values_[1] * values_[3]);

  const double inverse_scale = 1.0 / determinant_value;

  // Inverse formula:
  //   A^-1 = adj(A) / det(A)
  // where adj(A) is the transpose of the cofactor matrix.
  return Matrix3(cofactor00 * inverse_scale, cofactor10 * inverse_scale, cofactor20 * inverse_scale,
                 cofactor01 * inverse_scale, cofactor11 * inverse_scale, cofactor21 * inverse_scale,
                 cofactor02 * inverse_scale, cofactor12 * inverse_scale,
                 cofactor22 * inverse_scale);
}

std::optional<Vector3> Matrix3::solve(const Vector3 &right_hand_side,
                                      const SolveMethod method) const noexcept {
  switch (method) {
  case SolveMethod::Auto: {
    // Strategy:
    // 1) Try Cholesky first (fast/stable when matrix is symmetric positive-definite).
    // 2) Fallback to LU with partial pivoting for general non-singular matrices.
    // 3) Fallback to QR as a robust final path.
    const std::optional<Cholesky3> cholesky_decomposition = cholesky();
    if (cholesky_decomposition.has_value()) {
      const std::optional<Vector3> cholesky_solution =
          solve_cholesky_system(cholesky_decomposition.value(), right_hand_side);
      if (cholesky_solution.has_value()) {
        return cholesky_solution;
      }
    }

    const std::optional<LU3> lu_decomposition = lu();
    if (lu_decomposition.has_value()) {
      const std::optional<Vector3> lu_solution =
          solve_lu_system(lu_decomposition.value(), right_hand_side);
      if (lu_solution.has_value()) {
        return lu_solution;
      }
    }

    const std::optional<QR3> qr_decomposition = qr();
    if (qr_decomposition.has_value()) {
      return solve_qr_system(qr_decomposition.value(), right_hand_side);
    }

    return std::nullopt;
  }
  case SolveMethod::LU: {
    const std::optional<LU3> lu_decomposition = lu();
    if (!lu_decomposition.has_value()) {
      return std::nullopt;
    }
    return solve_lu_system(lu_decomposition.value(), right_hand_side);
  }
  case SolveMethod::QR: {
    const std::optional<QR3> qr_decomposition = qr();
    if (!qr_decomposition.has_value()) {
      return std::nullopt;
    }
    return solve_qr_system(qr_decomposition.value(), right_hand_side);
  }
  case SolveMethod::Cholesky: {
    const std::optional<Cholesky3> cholesky_decomposition = cholesky();
    if (!cholesky_decomposition.has_value()) {
      return std::nullopt;
    }
    return solve_cholesky_system(cholesky_decomposition.value(), right_hand_side);
  }
  }

  return std::nullopt;
}

std::optional<LU3> Matrix3::lu() const noexcept {
  // Doolittle-style LU factorization with partial pivoting.
  // We progressively eliminate entries below each pivot to form U.
  // Multipliers used for elimination are stored in L.
  // Row swaps are tracked as permutation P such that:
  //   P*A = L*U
  std::array<double, kMatrixEntryCount> lower_values{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, kMatrixEntryCount> upper_values = values_;
  std::array<std::size_t, 3> permutation_values{0U, 1U, 2U};
  int parity_value = 1;

  for (std::size_t pivot_column = 0U; pivot_column < kDimension; ++pivot_column) {
    // Partial pivoting: choose the largest-magnitude pivot in the active column.
    std::size_t pivot_row = pivot_column;
    double pivot_abs = std::fabs(upper_values[linear_index(pivot_row, pivot_column)]);

    for (std::size_t candidate_row = pivot_column + 1U; candidate_row < kDimension;
         ++candidate_row) {
      const double candidate_abs =
          std::fabs(upper_values[linear_index(candidate_row, pivot_column)]);
      if (candidate_abs > pivot_abs) {
        pivot_abs = candidate_abs;
        pivot_row = candidate_row;
      }
    }

    if (pivot_abs <= kNearZeroTolerance) {
      return std::nullopt;
    }

    if (pivot_row != pivot_column) {
      // Swap active rows in U so the largest available pivot is on the diagonal.
      for (std::size_t column_index = 0U; column_index < kDimension; ++column_index) {
        std::swap(upper_values[linear_index(pivot_column, column_index)],
                  upper_values[linear_index(pivot_row, column_index)]);
      }

      // Keep previously computed L multipliers aligned with the same row swaps.
      for (std::size_t previous_column = 0U; previous_column < pivot_column; ++previous_column) {
        std::swap(lower_values[linear_index(pivot_column, previous_column)],
                  lower_values[linear_index(pivot_row, previous_column)]);
      }

      std::swap(permutation_values[pivot_column], permutation_values[pivot_row]);
      parity_value = -parity_value;
    }

    const double pivot_value = upper_values[linear_index(pivot_column, pivot_column)];
    for (std::size_t row_index = pivot_column + 1U; row_index < kDimension; ++row_index) {
      // Elimination factor used to zero out U(row_index, pivot_column).
      const double factor_value = upper_values[linear_index(row_index, pivot_column)] / pivot_value;
      lower_values[linear_index(row_index, pivot_column)] = factor_value;
      upper_values[linear_index(row_index, pivot_column)] = 0.0;

      for (std::size_t column_index = pivot_column + 1U; column_index < kDimension;
           ++column_index) {
        upper_values[linear_index(row_index, column_index)] -=
            factor_value * upper_values[linear_index(pivot_column, column_index)];
      }
    }
  }

  return LU3{lower_values, upper_values, permutation_values, parity_value};
}

std::optional<QR3> Matrix3::qr() const noexcept {
  // Modified Gram-Schmidt orthogonalizes matrix columns.
  // Intuition:
  // - Take the first column as-is and normalize -> first orthonormal basis vector.
  // - Remove its projection from the second column, then normalize.
  // - Remove projections of first two basis vectors from third column, then normalize.
  //
  // This produces:
  //   A = Q*R
  // where Q has orthonormal columns and R is upper-triangular.
  const std::array<double, kDimension> column0{values_[0], values_[3], values_[6]};
  const std::array<double, kDimension> column1{values_[1], values_[4], values_[7]};
  const std::array<double, kDimension> column2{values_[2], values_[5], values_[8]};

  const std::array<double, kDimension> basis0 = column0;
  const double r00 = l2_norm3(basis0);
  if (r00 <= kNearZeroTolerance) {
    return std::nullopt;
  }
  const std::array<double, kDimension> orthogonal0 = scaled3(basis0, 1.0 / r00);

  const double r01 = dot3(orthogonal0, column1);
  const double r02 = dot3(orthogonal0, column2);

  const std::array<double, kDimension> basis1 = subtract3(column1, scaled3(orthogonal0, r01));
  const double r11 = l2_norm3(basis1);
  if (r11 <= kNearZeroTolerance) {
    return std::nullopt;
  }
  const std::array<double, kDimension> orthogonal1 = scaled3(basis1, 1.0 / r11);

  const double r12 = dot3(orthogonal1, column2);
  const std::array<double, kDimension> basis2 =
      subtract3(subtract3(column2, scaled3(orthogonal0, r02)), scaled3(orthogonal1, r12));
  const double r22 = l2_norm3(basis2);
  if (r22 <= kNearZeroTolerance) {
    return std::nullopt;
  }
  const std::array<double, kDimension> orthogonal2 = scaled3(basis2, 1.0 / r22);

  const std::array<double, kMatrixEntryCount> q_values{
      orthogonal0[0], orthogonal1[0], orthogonal2[0], orthogonal0[1], orthogonal1[1],
      orthogonal2[1], orthogonal0[2], orthogonal1[2], orthogonal2[2]};

  // R entries are projection magnitudes from Gram-Schmidt.
  const std::array<double, kMatrixEntryCount> r_values{r00, r01, r02, 0.0, r11, r12, 0.0, 0.0, r22};

  return QR3{q_values, r_values};
}

std::optional<Cholesky3> Matrix3::cholesky() const noexcept {
  // Cholesky factorization builds lower-triangular L such that:
  //   A = L * L^T
  // This only exists for symmetric positive-definite matrices.
  //
  // Each element is computed from previously solved entries:
  // - diagonal terms use sqrt(remaining_value)
  // - off-diagonal terms divide by a diagonal value from L
  std::array<double, kMatrixEntryCount> lower_values{};

  for (std::size_t row_index = 0U; row_index < kDimension; ++row_index) {
    for (std::size_t column_index = 0U; column_index <= row_index; ++column_index) {
      double sum_value = values_[linear_index(row_index, column_index)];
      for (std::size_t product_index = 0U; product_index < column_index; ++product_index) {
        // NOLINTNEXTLINE(readability-suspicious-call-argument): row-major indexing is (row, col).
        sum_value -= lower_values[linear_index(row_index, product_index)] *
                     lower_values[linear_index(column_index, product_index)];
      }

      if (row_index == column_index) {
        // Non-positive diagonal residual means the matrix is not SPD.
        if (sum_value <= kNearZeroTolerance) {
          return std::nullopt;
        }
        lower_values[linear_index(row_index, column_index)] = std::sqrt(sum_value);
      } else {
        const double diagonal_value = lower_values[linear_index(column_index, column_index)];
        if (std::fabs(diagonal_value) <= kNearZeroTolerance) {
          return std::nullopt;
        }
        lower_values[linear_index(row_index, column_index)] = sum_value / diagonal_value;
      }
    }
  }

  return Cholesky3{lower_values};
}

} // namespace cpp_helper_libs::linear_algebra
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,bugprone-easily-swappable-parameters,readability-suspicious-call-argument)
