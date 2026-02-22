// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <memory>

#include "cpp_helper_libs/linear_algebra/decompositions.hpp"
#include "cpp_helper_libs/linear_algebra/matrix3.hpp"
#include "test_helpers.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

using cpp_helper_libs::linear_algebra::Cholesky3;
using cpp_helper_libs::linear_algebra::LU3;
using cpp_helper_libs::linear_algebra::Matrix3;
using cpp_helper_libs::linear_algebra::QR3;
using cpp_helper_libs::linear_algebra::test::expect_matrix3_near;

TEST(DecompositionsTest, LuMatricesMaterializeFromRowMajorStorage) {
  constexpr double kTolerance = 1e-12;

  const LU3 decomposition{
      .lower = {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 4.0, 1.0},
      .upper = {5.0, 6.0, 7.0, 0.0, 8.0, 9.0, 0.0, 0.0, 10.0},
      .permutation = {0U, 1U, 2U},
      .parity = 1,
  };

  expect_matrix3_near(decomposition.lower_matrix(),
                      Matrix3(1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 4.0, 1.0), kTolerance);
  expect_matrix3_near(decomposition.upper_matrix(),
                      Matrix3(5.0, 6.0, 7.0, 0.0, 8.0, 9.0, 0.0, 0.0, 10.0), kTolerance);
}

TEST(DecompositionsTest, QrMatricesMaterializeFromRowMajorStorage) {
  constexpr double kTolerance = 1e-12;

  const QR3 decomposition{
      .orthogonal = {1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0},
      .upper = {2.0, 3.0, 4.0, 0.0, 5.0, 6.0, 0.0, 0.0, 7.0},
  };

  expect_matrix3_near(decomposition.orthogonal_matrix(),
                      Matrix3(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0), kTolerance);
  expect_matrix3_near(decomposition.upper_matrix(),
                      Matrix3(2.0, 3.0, 4.0, 0.0, 5.0, 6.0, 0.0, 0.0, 7.0), kTolerance);
}

TEST(DecompositionsTest, CholeskyMatrixMaterializesFromRowMajorStorage) {
  constexpr double kTolerance = 1e-12;

  const Cholesky3 decomposition{
      .lower = {1.0, 0.0, 0.0, 2.0, 3.0, 0.0, 4.0, 5.0, 6.0},
  };

  expect_matrix3_near(decomposition.lower_matrix(),
                      Matrix3(1.0, 0.0, 0.0, 2.0, 3.0, 0.0, 4.0, 5.0, 6.0), kTolerance);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
