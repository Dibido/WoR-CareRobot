#include "kinematics/DenavitHartenberg.hpp"
#include "gtest/gtest.h"

TEST(DenavitHartenberg, ForwardKinematics)
{

  std::vector<kinematics::Link> jointsSimplify;
  std::vector<double> bigTheta = { 0, 0};

  jointsSimplify.push_back(
      kinematics::Link(0, 0, 2, kinematics::eJoint::REVOLUTE, -M_PI, M_PI));
  jointsSimplify.push_back(kinematics::Link(
      0, -M_PI_2, 0, kinematics::eJoint::REVOLUTE, -M_PI, M_PI));
  jointsSimplify.push_back(kinematics::Link(
      0, M_PI_2, 2, 0, kinematics::eJoint::STATIC));

  kinematics::DenavitHartenberg denavitHartenberg(jointsSimplify);

  const auto endEffector = denavitHartenberg.forwardKinematicsYPR(bigTheta);
  const Matrix<double, 6, 1> expectedEndEffector{ 0, 0, 4, 0, 0, 0 };

  EXPECT_EQ(true, equals(expectedEndEffector,
                         denavitHartenberg.forwardKinematicsYPR(bigTheta),
                         std::numeric_limits<double>::epsilon(), 10));

  std::vector<double> bigTheta2 = { 0, M_PI_2};

  const Matrix<double, 6, 1> expectedEndEffector2{ 2, 0, 2, M_PI_4, M_PI_2, M_PI_4 };
  EXPECT_EQ(true, equals(expectedEndEffector2,
                         denavitHartenberg.forwardKinematicsYPR(bigTheta2),
                         std::numeric_limits<double>::epsilon(), 10));
}