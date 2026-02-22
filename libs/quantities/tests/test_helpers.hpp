#ifndef CPP_HELPER_LIBS_QUANTITIES_TESTS_TEST_HELPERS_HPP
#define CPP_HELPER_LIBS_QUANTITIES_TESTS_TEST_HELPERS_HPP

#include <cstring>
#include <limits>
#include <type_traits>

namespace cpp_helper_libs::quantities::test {

template <typename Enum> Enum make_invalid_enum() {
  Enum value{};
  using Underlying = std::underlying_type_t<Enum>;
  constexpr Underlying kInvalidRaw = std::numeric_limits<Underlying>::max();
  static_assert(sizeof(Underlying) == sizeof(Enum));
  std::memcpy(&value, &kInvalidRaw, sizeof(Underlying));
  return value;
}

} // namespace cpp_helper_libs::quantities::test

#endif // CPP_HELPER_LIBS_QUANTITIES_TESTS_TEST_HELPERS_HPP
