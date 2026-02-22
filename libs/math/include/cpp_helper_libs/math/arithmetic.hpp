// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_MATH_ARITHMETIC_HPP
#define CPP_HELPER_LIBS_MATH_ARITHMETIC_HPP

namespace cpp_helper_libs::math {

/**
 * @brief Add two integers.
 *
 * @param lhs Left-hand operand.
 * @param rhs Right-hand operand.
 * @return Sum of @p lhs and @p rhs.
 */
int add(int lhs, int rhs);

/**
 * @brief Subtract one integer from another.
 *
 * @param lhs Left-hand operand (minuend).
 * @param rhs Right-hand operand (subtrahend).
 * @return Difference @p lhs - @p rhs.
 */
int sub(int lhs, int rhs);

} // namespace cpp_helper_libs::math

#endif // CPP_HELPER_LIBS_MATH_ARITHMETIC_HPP
