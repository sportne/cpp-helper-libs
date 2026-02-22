// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "cpp_helper_libs/linear_algebra/matrix.hpp"
#include "test_helpers.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

using cpp_helper_libs::linear_algebra::Matrix;
using cpp_helper_libs::linear_algebra::test::require_value;

TEST(MatrixTest, FromRowMajorValidatesShape) {
  const std::optional<Matrix> valid =
      Matrix::from_row_major(2U, 3U, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
  const std::optional<Matrix> invalid = Matrix::from_row_major(2U, 3U, {1.0, 2.0});

  const Matrix valid_value = require_value(valid);
  EXPECT_EQ(valid_value.rows(), 2U);
  EXPECT_EQ(valid_value.cols(), 3U);
  EXPECT_DOUBLE_EQ(valid_value(1U, 2U), 6.0);
  EXPECT_FALSE(invalid.has_value());
}

TEST(MatrixTest, SupportsFactoriesAndDeterministicRandom) {
  const Matrix zeros = Matrix::zeros(2U, 2U);
  EXPECT_DOUBLE_EQ(zeros(0U, 0U), 0.0);
  EXPECT_DOUBLE_EQ(zeros(1U, 1U), 0.0);

  const Matrix ones = Matrix::ones(2U, 2U);
  EXPECT_DOUBLE_EQ(ones(0U, 1U), 1.0);

  const Matrix identity = Matrix::identity(3U);
  EXPECT_DOUBLE_EQ(identity(0U, 0U), 1.0);
  EXPECT_DOUBLE_EQ(identity(0U, 1U), 0.0);
  EXPECT_DOUBLE_EQ(identity(2U, 2U), 1.0);

  const Matrix first_random = Matrix::random(2U, 3U, 42U, -1.0, 1.0);
  const Matrix second_random = Matrix::random(2U, 3U, 42U, -1.0, 1.0);
  EXPECT_EQ(first_random, second_random);
}

TEST(MatrixTest, ShapeMismatchReturnsNullopt) {
  const Matrix lhs = require_value(Matrix::from_row_major(2U, 2U, {1.0, 2.0, 3.0, 4.0}));
  const Matrix rhs = require_value(Matrix::from_row_major(3U, 1U, {1.0, 2.0, 3.0}));

  EXPECT_FALSE(lhs.add(rhs).has_value());
  EXPECT_FALSE(lhs.subtract(rhs).has_value());
  EXPECT_FALSE(lhs.hadamard_product(rhs).has_value());
  EXPECT_FALSE(lhs.hadamard_divide(rhs).has_value());
  EXPECT_FALSE(lhs.dot(rhs).has_value());
}

TEST(MatrixTest, PerformsDynamicArithmeticAndMultiplication) {
  constexpr double kTolerance = 1e-12;

  const Matrix lhs = require_value(Matrix::from_row_major(2U, 3U, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
  const Matrix rhs = require_value(Matrix::from_row_major(2U, 3U, {6.0, 5.0, 4.0, 3.0, 2.0, 1.0}));
  const Matrix rhs_product =
      require_value(Matrix::from_row_major(3U, 2U, {7.0, 8.0, 9.0, 10.0, 11.0, 12.0}));

  const Matrix added = require_value(lhs.add(rhs));
  EXPECT_DOUBLE_EQ(added(0U, 0U), 7.0);
  EXPECT_DOUBLE_EQ(added(1U, 2U), 7.0);

  const Matrix hadamard = require_value(lhs.hadamard_product(rhs));
  EXPECT_DOUBLE_EQ(hadamard(0U, 0U), 6.0);
  EXPECT_DOUBLE_EQ(hadamard(1U, 2U), 6.0);

  const Matrix product = require_value(lhs.multiply(rhs_product));
  EXPECT_DOUBLE_EQ(product(0U, 0U), 58.0);
  EXPECT_DOUBLE_EQ(product(0U, 1U), 64.0);
  EXPECT_DOUBLE_EQ(product(1U, 0U), 139.0);
  EXPECT_DOUBLE_EQ(product(1U, 1U), 154.0);

  const double dot_value = require_value(lhs.dot(lhs));
  EXPECT_NEAR(dot_value, 91.0, kTolerance);
}

TEST(MatrixTest, ComputesNormsAndTranspose) {
  constexpr double kTolerance = 1e-12;

  const Matrix value =
      require_value(Matrix::from_row_major(2U, 3U, {1.0, -2.0, 3.0, -4.0, 5.0, -6.0}));

  EXPECT_DOUBLE_EQ(value.l1_norm(), 21.0);
  EXPECT_NEAR(value.l2_norm(), std::sqrt(91.0), kTolerance);
  EXPECT_DOUBLE_EQ(value.max_norm(), 6.0);

  const Matrix transposed = value.transpose();
  EXPECT_EQ(transposed.rows(), 3U);
  EXPECT_EQ(transposed.cols(), 2U);
  EXPECT_DOUBLE_EQ(transposed(0U, 1U), -4.0);
  EXPECT_DOUBLE_EQ(transposed(2U, 0U), 3.0);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
