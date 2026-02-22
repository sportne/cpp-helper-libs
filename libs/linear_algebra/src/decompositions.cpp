// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT
#include "cpp_helper_libs/linear_algebra/decompositions.hpp"

#include "cpp_helper_libs/linear_algebra/matrix3.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace cpp_helper_libs::linear_algebra {

namespace {

// Rebuild a Matrix3 object from a flat row-major array carried in decomposition results.
// The decomposition structs keep arrays to avoid circular include issues in headers.
Matrix3 matrix_from_row_major(const std::array<double, 9> &values) noexcept {
  return {values[0], values[1], values[2], values[3], values[4],
          values[5], values[6], values[7], values[8]};
}

} // namespace

Matrix3 LU3::lower_matrix() const noexcept { return matrix_from_row_major(lower); }

Matrix3 LU3::upper_matrix() const noexcept { return matrix_from_row_major(upper); }

Matrix3 QR3::orthogonal_matrix() const noexcept { return matrix_from_row_major(orthogonal); }

Matrix3 QR3::upper_matrix() const noexcept { return matrix_from_row_major(upper); }

Matrix3 Cholesky3::lower_matrix() const noexcept { return matrix_from_row_major(lower); }

} // namespace cpp_helper_libs::linear_algebra
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
