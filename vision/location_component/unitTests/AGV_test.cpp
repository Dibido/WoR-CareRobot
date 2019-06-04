// Bring in gtest
#include "location_component/AGV.hpp"
#include <exception>
#include <gtest/gtest.h>

const float cValidSpeed_m_s = 1.0;
const float cInvalidSpeed_m_s = -1.0;

TEST(AGVSuite, AGVConstructorSpeedBelowZero)
{
  EXPECT_NO_THROW(location_component::AGV{ cValidSpeed_m_s });
  EXPECT_THROW(location_component::AGV{ cInvalidSpeed_m_s }, std::range_error);
}

TEST(AGVSuite, AGVReferenceSpeedBelowZero)
{
  location_component::AGV lAGV{ cValidSpeed_m_s };
  // The assertion is made before the reference is returned.
  // That's why the assertion doesn't trigger until the second reference return.
  ASSERT_NO_THROW(lAGV.speed() = cInvalidSpeed_m_s);
  ASSERT_THROW(lAGV.speed() = cInvalidSpeed_m_s, std::range_error);
}

TEST(AGVSuite, AGVConstReferenceSpeedBelowZero)
{
  location_component::AGV lAGV{ cValidSpeed_m_s };
  ASSERT_NO_THROW(lAGV.speed());
  lAGV.speed() = cInvalidSpeed_m_s;
  ASSERT_THROW(lAGV.speed(), std::range_error);
}
