#include "kinematics/Configuration.hpp"
#include <gtest/gtest.h>

namespace kinematics
{
  TEST(Configuration, Constructors)
  {
    Configuration configuration;

    EXPECT_EQ(false, configuration.result());

    for (std::size_t i = 0; i < configuration.size; ++i)
    {
      EXPECT_EQ(0, configuration[i]);
    }
  }

  TEST(Configuration, SettersAndGetters)
  {
    Configuration configuration;

    configuration.setResult(true);
    EXPECT_EQ(true, configuration.result());

    configuration.setTheta(2, 3);
    EXPECT_DOUBLE_EQ(3, configuration[2]);

    configuration.setTheta(1, M_PI_2 + M_PI);
    EXPECT_DOUBLE_EQ(-M_PI_2, configuration[1]);

    EXPECT_ANY_THROW(configuration.setTheta(cKinematicsDoF + 1, 2));

    EXPECT_ANY_THROW(configuration[20]);
  }
} // namespace kinematics
