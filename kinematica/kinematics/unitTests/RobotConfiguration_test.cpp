#include "kinematics/RobotConfiguration.hpp"
#include <gtest/gtest.h>

namespace kinematics
{
  TEST(Configuration, Getters)
  {
    RobotConfiguration robotConfiguration;

    EXPECT_NO_THROW(robotConfiguration[2]);

    EXPECT_ANY_THROW(robotConfiguration[20]);
  }
} // namespace kinematics
