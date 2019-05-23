#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <gtest/gtest.h>

namespace kinematics
{

  TEST(DenavitHartenberg, InverseKinematicsSuccess)
  {
    DenavitHartenberg denavitHartenberg;
    Configuration currentBigTheta;
    Configuration goaaBigTheta;
    goaaBigTheta.setTheta(0, 0);
    goaaBigTheta.setTheta(1, M_PI * 1.2);
    goaaBigTheta.setTheta(2, M_PI * 0.7);
    goaaBigTheta.setTheta(3, M_PI_4);
    goaaBigTheta.setTheta(4, M_PI * 0.3);
    goaaBigTheta.setTheta(5, M_PI_4);
    goaaBigTheta.setTheta(6, 0);

    Matrix<double, 6, 1> aGoalEndEffector =
        denavitHartenberg.forwardKinematicsYPR(goaaBigTheta);

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
    const Matrix<double, 6, 1> expectedEndEffector{ 0.088, 0.0, 0.9259999999,
                                                    0.0,   0.0, 3.141592653 };

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
} // namespace kinematics