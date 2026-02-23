#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <stdexcept>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

template <typename ValueType> ValueType require_value(const std::optional<ValueType> &candidate) {
  if (!candidate.has_value()) {
    throw std::runtime_error("Expected optional to contain a value");
  }

  return candidate.value();
}

using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::spherical_geometry::CurveIntersection;
using cpp_helper_libs::spherical_geometry::CurveIntersectionKind;
using cpp_helper_libs::spherical_geometry::CurveLocation;

TEST(IntersectionTest, BuildsPointAndEndpointTouchRecords) {
  const UnitVector3 point = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const CurveLocation first_location{.parameter = 0.25, .at_start = false, .at_end = false};
  const CurveLocation second_location{.parameter = 0.5, .at_start = true, .at_end = false};

  const CurveIntersection point_record =
      CurveIntersection::point(point, first_location, second_location);
  EXPECT_EQ(point_record.kind(), CurveIntersectionKind::Point);
  EXPECT_EQ(point_record.first_point(), point);
  EXPECT_FALSE(point_record.second_point().has_value());
  EXPECT_DOUBLE_EQ(point_record.first_curve_first_location().parameter, 0.25);
  EXPECT_TRUE(point_record.second_curve_first_location().at_start);
  EXPECT_FALSE(point_record.first_curve_second_location().has_value());
  EXPECT_FALSE(point_record.second_curve_second_location().has_value());

  const CurveIntersection touch_record =
      CurveIntersection::endpoint_touch(point, first_location, second_location);
  EXPECT_EQ(touch_record.kind(), CurveIntersectionKind::EndpointTouch);
  EXPECT_EQ(touch_record.first_point(), point);
  EXPECT_DOUBLE_EQ(touch_record.first_curve_first_location().parameter, 0.25);
  EXPECT_DOUBLE_EQ(touch_record.second_curve_first_location().parameter, 0.5);
}

TEST(IntersectionTest, BuildsOverlapSegmentRecords) {
  const UnitVector3 first_point = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 second_point = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));

  const CurveLocation first_first{.parameter = 0.0, .at_start = true, .at_end = false};
  const CurveLocation first_second{.parameter = 1.0, .at_start = false, .at_end = true};
  const CurveLocation second_first{.parameter = 0.2, .at_start = false, .at_end = false};
  const CurveLocation second_second{.parameter = 0.8, .at_start = false, .at_end = false};

  const CurveIntersection record = CurveIntersection::overlap_segment(
      first_point, second_point, first_first, first_second, second_first, second_second);

  EXPECT_EQ(record.kind(), CurveIntersectionKind::OverlapSegment);
  EXPECT_EQ(record.first_point(), first_point);
  const auto second_point_candidate = record.second_point();
  ASSERT_TRUE(second_point_candidate.has_value());
  EXPECT_EQ(require_value(second_point_candidate), second_point);
  const auto first_second_location_candidate = record.first_curve_second_location();
  ASSERT_TRUE(first_second_location_candidate.has_value());
  const auto second_second_location_candidate = record.second_curve_second_location();
  ASSERT_TRUE(second_second_location_candidate.has_value());
  EXPECT_TRUE(record.first_curve_first_location().at_start);
  EXPECT_TRUE(require_value(first_second_location_candidate).at_end);
  EXPECT_DOUBLE_EQ(record.second_curve_first_location().parameter, 0.2);
  EXPECT_DOUBLE_EQ(require_value(second_second_location_candidate).parameter, 0.8);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
