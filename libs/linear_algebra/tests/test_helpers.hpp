// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_TESTS_TEST_HELPERS_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_TESTS_TEST_HELPERS_HPP

#include <cstddef>
#include <optional>
#include <stdexcept>

#include <gtest/gtest.h>

#include "cpp_helper_libs/linear_algebra/matrix3.hpp"

namespace cpp_helper_libs::linear_algebra::test {

template <typename ValueType> ValueType require_value(const std::optional<ValueType> &candidate) {
  if (!candidate.has_value()) {
    throw std::runtime_error("Expected optional to contain a value");
  }

  return candidate.value();
}

inline void expect_matrix3_near(const Matrix3 &actual_value, const Matrix3 &expected_value,
                                const double tolerance) {
  for (std::size_t row_index = 0U; row_index < 3U; ++row_index) {
    for (std::size_t column_index = 0U; column_index < 3U; ++column_index) {
      EXPECT_NEAR(actual_value(row_index, column_index), expected_value(row_index, column_index),
                  tolerance);
    }
  }
}

} // namespace cpp_helper_libs::linear_algebra::test

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_TESTS_TEST_HELPERS_HPP
