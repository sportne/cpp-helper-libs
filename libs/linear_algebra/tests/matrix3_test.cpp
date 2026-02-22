// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>

#include "cpp_helper_libs/linear_algebra/decompositions.hpp"
#include "cpp_helper_libs/linear_algebra/matrix3.hpp"
#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "test_helpers.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

using cpp_helper_libs::linear_algebra::Cholesky3;
using cpp_helper_libs::linear_algebra::LU3;
using cpp_helper_libs::linear_algebra::Matrix3;
using cpp_helper_libs::linear_algebra::QR3;
using cpp_helper_libs::linear_algebra::SolveMethod;
using cpp_helper_libs::linear_algebra::Vector3;
using cpp_helper_libs::linear_algebra::test::expect_matrix3_near;
using cpp_helper_libs::linear_algebra::test::require_value;

TEST(Matrix3Test, ConstructsAndAccessesElements) {
  const Matrix3 value(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

  EXPECT_DOUBLE_EQ(value(0U, 0U), 1.0);
  EXPECT_DOUBLE_EQ(value(1U, 2U), 6.0);
  EXPECT_DOUBLE_EQ(value(2U, 1U), 8.0);
}

TEST(Matrix3Test, SupportsFactoryConstructionAndDeterministicRandom) {
  EXPECT_EQ(Matrix3::zeros(), Matrix3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_EQ(Matrix3::ones(), Matrix3(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
  EXPECT_EQ(Matrix3::identity(), Matrix3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0));

  const Matrix3 first_random = Matrix3::random(1337U, -2.0, 2.0);
  const Matrix3 second_random = Matrix3::random(1337U, -2.0, 2.0);
  EXPECT_EQ(first_random, second_random);
}

TEST(Matrix3Test, SupportsArithmeticAndHadamardOperations) {
  const Matrix3 lhs(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  const Matrix3 rhs(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);

  EXPECT_EQ(lhs + rhs, Matrix3(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0));
  EXPECT_EQ(lhs - rhs, Matrix3(-8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0));
  EXPECT_EQ(-lhs, Matrix3(-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0));
  EXPECT_EQ(lhs * 2.0, Matrix3(2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0));
  EXPECT_EQ(2.0 * lhs, Matrix3(2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0));

  EXPECT_EQ(lhs.hadamard_product(rhs), Matrix3(9.0, 16.0, 21.0, 24.0, 25.0, 24.0, 21.0, 16.0, 9.0));

  const Matrix3 divisors(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  const Matrix3 divided = require_value(lhs.hadamard_divide(divisors));
  EXPECT_EQ(divided, Matrix3(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
}

TEST(Matrix3Test, HadamardDivisionReturnsNulloptOnZeroDivisor) {
  const Matrix3 lhs = Matrix3::ones();
  const Matrix3 divisors(1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0);

  EXPECT_FALSE(lhs.hadamard_divide(divisors).has_value());
}

TEST(Matrix3Test, SupportsDotAndMatrixProducts) {
  constexpr double kTolerance = 1e-12;

  const Matrix3 lhs(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  const Matrix3 rhs(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);
  const Vector3 vector(1.0, 0.0, -1.0);

  EXPECT_DOUBLE_EQ(lhs.dot(rhs), 165.0);
  EXPECT_EQ(lhs.multiply(vector), Vector3(-2.0, -2.0, -2.0));

  const Matrix3 product = lhs.multiply(rhs);
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
  expect_matrix3_near(product, Matrix3(30.0, 24.0, 18.0, 84.0, 69.0, 54.0, 138.0, 114.0, 90.0),
                      kTolerance);
}

TEST(Matrix3Test, ComputesNormsAndInvariants) {
  constexpr double kTolerance = 1e-12;

  const Matrix3 value(1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0, -8.0, 9.0);

  EXPECT_DOUBLE_EQ(value.l1_norm(), 45.0);
  EXPECT_NEAR(value.l2_norm(), std::sqrt(285.0), kTolerance);
  EXPECT_DOUBLE_EQ(value.max_norm(), 9.0);
  EXPECT_EQ(value.transpose(), Matrix3(1.0, -4.0, 7.0, -2.0, 5.0, -8.0, 3.0, -6.0, 9.0));
  EXPECT_DOUBLE_EQ(value.trace(), 15.0);
  EXPECT_DOUBLE_EQ(value.determinant(), 0.0);
}

TEST(Matrix3Test, InverseComputesExpectedValueAndRejectsSingular) {
  constexpr double kTolerance = 1e-12;

  const Matrix3 invertible(4.0, 7.0, 2.0, 3.0, 6.0, 1.0, 2.0, 5.0, 1.0);
  const Matrix3 inverse = require_value(invertible.inverse());
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
  expect_matrix3_near(
      inverse, Matrix3(1.0 / 3.0, 1.0, -5.0 / 3.0, -1.0 / 3.0, 0.0, 2.0 / 3.0, 1.0, -2.0, 1.0),
      kTolerance);

  const Matrix3 singular(1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 3.0, 6.0, 9.0);
  EXPECT_FALSE(singular.inverse().has_value());
}

TEST(Matrix3Test, LuDecompositionSatisfiesPaEqualsLu) {
  constexpr double kTolerance = 1e-9;

  const Matrix3 matrix_value(2.0, 1.0, 1.0, 4.0, -6.0, 0.0, -2.0, 7.0, 2.0);

  const LU3 decomposition = require_value(matrix_value.lu());
  const Matrix3 lower = decomposition.lower_matrix();
  const Matrix3 upper = decomposition.upper_matrix();

  Matrix3 permutation_applied = Matrix3::zeros();
  for (std::size_t row_index = 0U; row_index < 3U; ++row_index) {
    const std::size_t source_row = decomposition.permutation.at(row_index);
    permutation_applied =
        permutation_applied + Matrix3(row_index == 0U ? matrix_value(source_row, 0U) : 0.0,
                                      row_index == 0U ? matrix_value(source_row, 1U) : 0.0,
                                      row_index == 0U ? matrix_value(source_row, 2U) : 0.0,
                                      row_index == 1U ? matrix_value(source_row, 0U) : 0.0,
                                      row_index == 1U ? matrix_value(source_row, 1U) : 0.0,
                                      row_index == 1U ? matrix_value(source_row, 2U) : 0.0,
                                      row_index == 2U ? matrix_value(source_row, 0U) : 0.0,
                                      row_index == 2U ? matrix_value(source_row, 1U) : 0.0,
                                      row_index == 2U ? matrix_value(source_row, 2U) : 0.0);
  }

  const Matrix3 reconstructed = lower.multiply(upper);
  expect_matrix3_near(reconstructed, permutation_applied, kTolerance);
}

TEST(Matrix3Test, QrDecompositionSatisfiesOrthogonalityAndReconstruction) {
  constexpr double kTolerance = 1e-9;

  const Matrix3 matrix_value(12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0);

  const QR3 decomposition = require_value(matrix_value.qr());
  const Matrix3 orthogonal = decomposition.orthogonal_matrix();
  const Matrix3 upper = decomposition.upper_matrix();

  const Matrix3 orthogonality = orthogonal.transpose().multiply(orthogonal);
  expect_matrix3_near(orthogonality, Matrix3::identity(), kTolerance);

  const Matrix3 reconstructed = orthogonal.multiply(upper);
  expect_matrix3_near(reconstructed, matrix_value, kTolerance);
}

TEST(Matrix3Test, CholeskyWorksForSpdAndRejectsNonSpd) {
  constexpr double kTolerance = 1e-9;

  const Matrix3 spd_matrix(4.0, 12.0, -16.0, 12.0, 37.0, -43.0, -16.0, -43.0, 98.0);

  const Cholesky3 decomposition = require_value(spd_matrix.cholesky());
  const Matrix3 lower = decomposition.lower_matrix();
  const Matrix3 reconstructed = lower.multiply(lower.transpose());
  expect_matrix3_near(reconstructed, spd_matrix, kTolerance);

  const Matrix3 non_spd(1.0, 2.0, 3.0, 2.0, 1.0, 4.0, 3.0, 4.0, 1.0);
  EXPECT_FALSE(non_spd.cholesky().has_value());
}

TEST(Matrix3Test, SolveHandlesMethodsAndSingularSystems) {
  constexpr double kTolerance = 1e-9;

  const Matrix3 matrix_value(4.0, 1.0, 2.0, 1.0, 5.0, 1.0, 2.0, 1.0, 3.0);
  const Vector3 rhs(7.0, 8.0, 5.0);

  const std::optional<Vector3> auto_solution = matrix_value.solve(rhs, SolveMethod::Auto);
  const std::optional<Vector3> lu_solution = matrix_value.solve(rhs, SolveMethod::LU);
  const std::optional<Vector3> qr_solution = matrix_value.solve(rhs, SolveMethod::QR);
  const std::optional<Vector3> cholesky_solution = matrix_value.solve(rhs, SolveMethod::Cholesky);

  const Vector3 auto_value = require_value(auto_solution);
  const Vector3 lu_value = require_value(lu_solution);
  const Vector3 qr_value = require_value(qr_solution);
  const Vector3 cholesky_value = require_value(cholesky_solution);

  EXPECT_NEAR(auto_value.x(), 1.2162162162162162, kTolerance);
  EXPECT_NEAR(auto_value.y(), 1.2702702702702702, kTolerance);
  EXPECT_NEAR(auto_value.z(), 0.43243243243243246, kTolerance);

  EXPECT_NEAR(lu_value.x(), auto_value.x(), kTolerance);
  EXPECT_NEAR(lu_value.y(), auto_value.y(), kTolerance);
  EXPECT_NEAR(lu_value.z(), auto_value.z(), kTolerance);

  EXPECT_NEAR(qr_value.x(), auto_value.x(), kTolerance);
  EXPECT_NEAR(qr_value.y(), auto_value.y(), kTolerance);
  EXPECT_NEAR(qr_value.z(), auto_value.z(), kTolerance);

  EXPECT_NEAR(cholesky_value.x(), auto_value.x(), kTolerance);
  EXPECT_NEAR(cholesky_value.y(), auto_value.y(), kTolerance);
  EXPECT_NEAR(cholesky_value.z(), auto_value.z(), kTolerance);

  const Matrix3 singular(1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 3.0, 6.0, 9.0);
  EXPECT_FALSE(singular.solve(Vector3(1.0, 1.0, 1.0), SolveMethod::Auto).has_value());
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
