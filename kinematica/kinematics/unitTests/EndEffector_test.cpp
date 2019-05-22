#include "kinematics/EndEffector.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace kinematics
{
  TEST(EndEffector, Constructor)
  {
    EndEffector endEffector(2, 4, 5, 3, M_PI + M_PI_2, M_PI_4);

    EXPECT_DOUBLE_EQ(2, endEffector.cX_m);
    EXPECT_DOUBLE_EQ(4, endEffector.cY_m);
    EXPECT_DOUBLE_EQ(5, endEffector.cZ_m);
    EXPECT_DOUBLE_EQ(3, endEffector.cYaw_rad);
    EXPECT_DOUBLE_EQ(-M_PI_2, endEffector.cPitch_rad);
    EXPECT_DOUBLE_EQ(M_PI_4, endEffector.cRoll_rad);
  }
} // namespace kinematics
