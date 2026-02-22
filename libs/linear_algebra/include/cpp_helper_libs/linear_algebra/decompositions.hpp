// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_LINEAR_ALGEBRA_DECOMPOSITIONS_HPP
#define CPP_HELPER_LIBS_LINEAR_ALGEBRA_DECOMPOSITIONS_HPP

#include <array>
#include <cstddef>

namespace cpp_helper_libs::linear_algebra {

class Matrix3;

/**
 * @brief Solver strategy for `Matrix3::solve`.
 */
enum class SolveMethod {
  Auto,
  LU,
  QR,
  Cholesky,
};

/**
 * @brief LU decomposition result for a 3x3 matrix with partial pivoting.
 *
 * The decomposition satisfies `P * A = L * U`, where permutation rows are encoded in
 * `permutation`.
 */
struct LU3 {
  /**
   * @brief Row-major entries of lower-triangular matrix L.
   */
  std::array<double, 9> lower;

  /**
   * @brief Row-major entries of upper-triangular matrix U.
   */
  std::array<double, 9> upper;

  /**
   * @brief Row permutation indices encoding matrix P.
   */
  std::array<std::size_t, 3> permutation;

  /**
   * @brief Permutation parity (`+1` for even swaps, `-1` for odd swaps).
   */
  int parity;

  /**
   * @brief Materialize lower-triangular matrix L as `Matrix3`.
   *
   * @return Matrix3 value for L.
   */
  Matrix3 lower_matrix() const noexcept;

  /**
   * @brief Materialize upper-triangular matrix U as `Matrix3`.
   *
   * @return Matrix3 value for U.
   */
  Matrix3 upper_matrix() const noexcept;
};

/**
 * @brief QR decomposition result for a 3x3 matrix.
 *
 * The decomposition satisfies `A = Q * R`, where Q is orthogonal and R is upper triangular.
 */
struct QR3 {
  /**
   * @brief Row-major entries of orthogonal matrix Q.
   */
  std::array<double, 9> orthogonal;

  /**
   * @brief Row-major entries of upper-triangular matrix R.
   */
  std::array<double, 9> upper;

  /**
   * @brief Materialize orthogonal matrix Q as `Matrix3`.
   *
   * @return Matrix3 value for Q.
   */
  Matrix3 orthogonal_matrix() const noexcept;

  /**
   * @brief Materialize upper-triangular matrix R as `Matrix3`.
   *
   * @return Matrix3 value for R.
   */
  Matrix3 upper_matrix() const noexcept;
};

/**
 * @brief Cholesky factorization result for a 3x3 matrix.
 *
 * The factorization satisfies `A = L * L^T`.
 */
struct Cholesky3 {
  /**
   * @brief Row-major entries of lower-triangular matrix L.
   */
  std::array<double, 9> lower;

  /**
   * @brief Materialize lower-triangular matrix L as `Matrix3`.
   *
   * @return Matrix3 value for L.
   */
  Matrix3 lower_matrix() const noexcept;
};

} // namespace cpp_helper_libs::linear_algebra

#endif // CPP_HELPER_LIBS_LINEAR_ALGEBRA_DECOMPOSITIONS_HPP
