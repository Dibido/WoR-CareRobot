#include "kinematics/RobotConfiguration.hpp"
#include <gtest/gtest.h>

namespace kinematics
{
  TEST(RobotConfiguration, Getters)
  {
    RobotConfiguration robotConfiguration;

    EXPECT_NO_THROW(robotConfiguration[2]);

    EXPECT_ANY_THROW(robotConfiguration[20]);
  }

  TEST(RobotConfiguration, ValidConfiguration)
  {
    RobotConfiguration robotConfiguration;

    Configuration configuration;

    // Joint 1 is invalid
    configuration.setTheta(0, M_PI);
    configuration.setTheta(1, 0);
    configuration.setTheta(2, 0);
    configuration.setTheta(3, -M_PI_2);
    configuration.setTheta(4, 0);
    configuration.setTheta(5, 0);
    configuration.setTheta(6, 0);

    EXPECT_EQ(false, robotConfiguration.isValidConfiguration(configuration));

    configuration.setTheta(0, 0);
    configuration.setTheta(6, M_PI);
    EXPECT_EQ(false, robotConfiguration.isValidConfiguration(configuration));

    configuration.setTheta(0, -2.8972);
    configuration.setTheta(6, 2.8972);
    EXPECT_EQ(true, robotConfiguration.isValidConfiguration(configuration));
  }

  TEST(RobotConfiguration, RandomiseConfiguration)
  {
    RobotConfiguration robotConfiguration;
    Configuration configuration;

    configuration.setTheta(0, M_PI);
    configuration.setTheta(3, -M_PI);

    robotConfiguration.randomiseConfiguration(configuration);

    EXPECT_EQ(true, robotConfiguration.isValidConfiguration(configuration));
    EXPECT_NE(M_PI, configuration[0]);
    EXPECT_NE(-M_PI, configuration[3]);
  }
} // namespace kinematics
