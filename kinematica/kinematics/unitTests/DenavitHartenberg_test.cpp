#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <gtest/gtest.h>

using namespace kinematics;

std::vector<Link> createConfiguration()
{
  std::vector<Link> joints;
  // The official config, uses the modified Denavit-Hartenberg
  // parameters Joint 1
  joints.push_back(Link(0, 0, 0.333, eJoint::REVOLUTE, degree2Radian(-166),
                        degree2Radian(166)));
  // Joint 2
  joints.push_back(Link(0, -M_PI_2, 0.0, eJoint::REVOLUTE, degree2Radian(-101),
                        degree2Radian(101)));
  // Joint 3
  joints.push_back(Link(0.0, M_PI_2, 0.316, eJoint::REVOLUTE,
                        degree2Radian(-166), degree2Radian(166)));
  // Joint 4
  joints.push_back(Link(0.0825, M_PI_2, 0.0, eJoint::REVOLUTE,
                        degree2Radian(-176), degree2Radian(-4)));
  // Joint 5
  joints.push_back(Link(-0.0825, -M_PI_2, 0.384, eJoint::REVOLUTE,
                        degree2Radian(-166), degree2Radian(166)));
  // Joint 6
  joints.push_back(Link(0.0, M_PI_2, 0.0, eJoint::REVOLUTE, degree2Radian(-1),
                        degree2Radian(215)));
  // Joint 7 -> This joint and the next have been combined to simplify
  // this prototype
  joints.push_back(Link(0.088, M_PI_2, 0, eJoint::REVOLUTE, degree2Radian(-166),
                        degree2Radian(166)));
  // flange
  joints.push_back(Link(0.0, 0.0, 0.107, 0.0, eJoint::STATIC));
  return joints;
}

TEST(DenavitHartenberg, InverseKinematics)
{
  DenavitHartenberg denavitHartenberg(createConfiguration());
  std::vector<double> currentBigTheta = { 0, 0, 0, 0, 0, 0, 0 };
  std::vector<double> goalBigTheta = { 0,      M_PI * 1.2, M_PI * 0.7,
                                       M_PI_4, M_PI * 0.3, M_PI_4,
                                       0 };

  Matrix<double, 6, 1> goalEndEffector =
      denavitHartenberg.forwardKinematicsYPR(goalBigTheta);

  std::vector<double> foundConfiguration =
      denavitHartenberg.inverseKinematics(goalEndEffector, currentBigTheta);
  Matrix<double, 6, 1> foundEndEffector =
      denavitHartenberg.forwardKinematicsYPR(foundConfiguration);

  EXPECT_EQ(true, transformationMatrixEquals(goalEndEffector, foundEndEffector,
                                             cIkEpsilon_m, cIkEpsilon_rad,
                                             cDhTransformPosRadSplit));


  Matrix<double, 6, 1> impossibleEndEffector{1, 0, 0, 0, 0, 0};

  std::vector<double> foundConfiguration2 =
      denavitHartenberg.inverseKinematics(impossibleEndEffector, currentBigTheta);
  Matrix<double, 6, 1> foundEndEffector2 =
      denavitHartenberg.forwardKinematicsYPR(foundConfiguration2);

  EXPECT_EQ(false, transformationMatrixEquals(impossibleEndEffector, foundEndEffector2,
                                             cIkEpsilon_m, cIkEpsilon_rad,
                                             cDhTransformPosRadSplit));
  
}