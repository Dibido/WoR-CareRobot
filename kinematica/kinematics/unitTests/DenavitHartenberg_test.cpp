#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/RobotConfiguration.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <gtest/gtest.h>

namespace kinematics
{
  // 2.8586,-1.1657,-1.8204,-2.6137,-0.4859,0.8275,-0.9049

  TEST(DenavitHartenberg, InverseKinematicsSuccess)
  {
    DenavitHartenberg denavitHartenberg;
    Configuration currentBigTheta;

    currentBigTheta.setTheta(0, 2.8586);
    currentBigTheta.setTheta(1, -1.1657);
    currentBigTheta.setTheta(2, -1.8204);
    currentBigTheta.setTheta(3, -2.6137);
    currentBigTheta.setTheta(4, -0.4859);
    currentBigTheta.setTheta(5, 0.8275);
    currentBigTheta.setTheta(6, -0.9049);

    Matrix<double, 6, 1> aGoalEndEffector = { -0.0200, 0.2000, 0.2100,
                                              0.0000,  1.5708, 1.5708 };
    Configuration foundConfiguration =
        denavitHartenberg.inverseKinematics(aGoalEndEffector, currentBigTheta);
    Matrix<double, 6, 1> foundEndEffector =
        denavitHartenberg.forwardKinematicsYPR(foundConfiguration);

    EXPECT_EQ(true, transformationMatrixEquals(
                        aGoalEndEffector, foundEndEffector, cIkEpsilon_m,
                        cIkEpsilon_rad, cDhTransformPosRadSplit));
    EXPECT_EQ(true, foundConfiguration.result());
  }
  TEST(DenavitHartenberg, InverseKinematicsSuccess2)
  {
    DenavitHartenberg denavitHartenberg;
    Configuration currentBigTheta;
    Configuration goalBigTheta;
    goalBigTheta.setTheta(0, 0);
    goalBigTheta.setTheta(1, M_PI * 1.2);
    goalBigTheta.setTheta(2, M_PI * 0.7);
    goalBigTheta.setTheta(3, M_PI_4);
    goalBigTheta.setTheta(4, M_PI * 0.3);
    goalBigTheta.setTheta(5, M_PI_4);
    goalBigTheta.setTheta(6, 0);

    Matrix<double, 6, 1> aGoalEndEffector =
        denavitHartenberg.forwardKinematicsYPR(goalBigTheta);

    Configuration foundConfiguration =
        denavitHartenberg.inverseKinematics(aGoalEndEffector, currentBigTheta);
    Matrix<double, 6, 1> foundEndEffector =
        denavitHartenberg.forwardKinematicsYPR(foundConfiguration);

    EXPECT_EQ(true, transformationMatrixEquals(
                        aGoalEndEffector, foundEndEffector, cIkEpsilon_m,
                        cIkEpsilon_rad, cDhTransformPosRadSplit));
    EXPECT_EQ(true, foundConfiguration.result());
  }

  TEST(DenavitHartenberg, InverseKinematicsFail)
  {
    DenavitHartenberg denavitHartenberg;
    Configuration currentBigTheta;

    Matrix<double, 6, 1> impossibleEndEffector{ 1, 0, 0, 0, 0, 0 };

    Configuration foundConfiguration2 = denavitHartenberg.inverseKinematics(
        impossibleEndEffector, currentBigTheta);
    Matrix<double, 6, 1> foundEndEffector2 =
        denavitHartenberg.forwardKinematicsYPR(foundConfiguration2);

    EXPECT_EQ(false, transformationMatrixEquals(
                         impossibleEndEffector, foundEndEffector2, cIkEpsilon_m,
                         cIkEpsilon_rad, cDhTransformPosRadSplit));
    EXPECT_EQ(false, foundConfiguration2.result());
  }

  TEST(DenavitHartenberg, ForwardKinematics)
  {
    Configuration bigTheta;

    DenavitHartenberg denavitHartenberg;

    const auto endEffector = denavitHartenberg.forwardKinematicsYPR(bigTheta);
    const Matrix<double, 6, 1> expectedEndEffector{ 0.088, 0.0, 0.7759999999,
                                                    0.0,   0.0, -3.141592653 };

    EXPECT_EQ(true, equals(expectedEndEffector,
                           denavitHartenberg.forwardKinematicsYPR(bigTheta),
                           0.000000001));

    Configuration bigTheta2;
    bigTheta2.setTheta(0, M_PI / 5);
    bigTheta2.setTheta(1, M_PI / 3);
    bigTheta2.setTheta(2, M_PI / 2);
    bigTheta2.setTheta(3, M_PI * 1.5);
    bigTheta2.setTheta(4, M_PI * 1.7);
    bigTheta2.setTheta(5, M_PI / 8);
    bigTheta2.setTheta(6, M_PI / 2);

    const Matrix<double, 6, 1> expectedEndEffector2{
      0.03297219602, 0.520015779,  0.410671164,
      0.6283185307,  0.1047197551, 1.963495408
    };
    EXPECT_EQ(true, equals(expectedEndEffector2,
                           denavitHartenberg.forwardKinematicsYPR(bigTheta2),
                           0.000000001));
  }

  TEST(DenavitHartenberg, EnduranceTest)
  {
    DenavitHartenberg denavitHartenberg;
    Configuration currentBigTheta;
    currentBigTheta.setTheta(0, M_PI + M_PI_2);
    currentBigTheta.setTheta(1, M_PI + M_PI_2);
    currentBigTheta.setTheta(2, M_PI + M_PI_2);
    currentBigTheta.setTheta(3, M_PI + M_PI_2);
    currentBigTheta.setTheta(4, M_PI + M_PI_2);
    currentBigTheta.setTheta(5, M_PI + M_PI_2);
    currentBigTheta.setTheta(6, M_PI + M_PI_2);

    Configuration goalBigTheta;
    goalBigTheta.setTheta(0, 0);
    goalBigTheta.setTheta(1, M_PI * 1.2);
    goalBigTheta.setTheta(2, M_PI * 0.7);
    goalBigTheta.setTheta(3, M_PI_4);
    goalBigTheta.setTheta(4, M_PI * 0.3);
    goalBigTheta.setTheta(5, M_PI_4);
    goalBigTheta.setTheta(6, 0);

    RobotConfiguration robotConfig;

    std::size_t max_it = 500;
    for (std::size_t i = 0; i < max_it; ++i)
    {
      Configuration startConfig = currentBigTheta;
      robotConfig.randomiseConfiguration(startConfig);

      Matrix<double, 6, 1> aGoalEndEffector =
          denavitHartenberg.forwardKinematicsYPR(goalBigTheta);

      Configuration foundConfiguration =
          denavitHartenberg.inverseKinematics(aGoalEndEffector, startConfig);
      Matrix<double, 6, 1> foundEndEffector =
          denavitHartenberg.forwardKinematicsYPR(foundConfiguration);

      ASSERT_EQ(true, transformationMatrixEquals(
                          aGoalEndEffector, foundEndEffector, cIkEpsilon_m,
                          cIkEpsilon_rad, cDhTransformPosRadSplit));
      ASSERT_EQ(true, foundConfiguration.result());
    }
  }
} // namespace kinematics