// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_CONVERSION_COMMON_HPP
#define CPP_HELPER_LIBS_QUANTITIES_CONVERSION_COMMON_HPP

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace cpp_helper_libs::quantities::internal {

template <typename UnitType, std::size_t TableSize>
double
to_raw_from_scale_table(const double value, const UnitType unit,
                        const std::array<std::pair<UnitType, double>, TableSize> &scales_to_raw,
                        const std::string_view type_name) {
  const auto it = std::find_if(scales_to_raw.begin(), scales_to_raw.end(),
                               [unit](const auto &entry) { return entry.first == unit; });
  if (it != scales_to_raw.end()) {
    return value * it->second;
  }

  throw std::invalid_argument("Invalid " + std::string(type_name) + "::Unit value");
}

template <typename UnitType, std::size_t TableSize>
double
from_raw_with_scale_table(const double raw, const UnitType unit,
                          const std::array<std::pair<UnitType, double>, TableSize> &scales_to_raw,
                          const std::string_view type_name) {
  const auto it = std::find_if(scales_to_raw.begin(), scales_to_raw.end(),
                               [unit](const auto &entry) { return entry.first == unit; });
  if (it != scales_to_raw.end()) {
    return raw / it->second;
  }

  throw std::invalid_argument("Invalid " + std::string(type_name) + "::Unit value");
}

} // namespace cpp_helper_libs::quantities::internal

#define CPPHL_DEFINE_SCALED_QUANTITY_CORE_METHODS(QuantityType, ScaleTable)                        \
  QuantityType::QuantityType(const double value, const Unit unit)                                  \
      : QuantityBase(to_raw(value, unit)) {}                                                       \
                                                                                                   \
  double QuantityType::in(const Unit unit) const {                                                 \
    return internal::from_raw_with_scale_table(raw_value(), unit, ScaleTable, #QuantityType);      \
  }                                                                                                \
                                                                                                   \
  QuantityType QuantityType::from_raw(const double raw) noexcept { return QuantityType(raw); }     \
                                                                                                   \
  double QuantityType::to_raw(const double value, const Unit unit) {                               \
    return internal::to_raw_from_scale_table(value, unit, ScaleTable, #QuantityType);              \
  }

#endif // CPP_HELPER_LIBS_QUANTITIES_CONVERSION_COMMON_HPP
