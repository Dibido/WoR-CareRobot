#include "kinematics/Link.hpp"
#include <gtest/gtest.h>

TEST(Link, Constructors)
{
  kinematics::Link link0(1, M_PI, 3, 0, kinematics::eJoint::STATIC);

  EXPECT_EQ(1, link0.getA());
  EXPECT_EQ(M_PI, link0.getAlpha());
  EXPECT_EQ(3, link0.getD());
  EXPECT_EQ(0, link0.getTheta());
  EXPECT_EQ(kinematics::eJoint::STATIC, link0.getType());

  kinematics::Link revoluteLink(2, M_PI_2, 2, kinematics::eJoint::REVOLUTE, 0,
                                4);

  EXPECT_EQ(2, revoluteLink.getA());
  EXPECT_EQ(M_PI_2, revoluteLink.getAlpha());
  EXPECT_EQ(2, revoluteLink.getD());
  EXPECT_EQ(0, revoluteLink.getTheta());
  EXPECT_EQ(kinematics::eJoint::REVOLUTE, revoluteLink.getType());

  kinematics::Link prismaticLink(5, M_PI_4, M_PI, kinematics::eJoint::PRISMATIC,
                                 0, 20);

  EXPECT_EQ(5, prismaticLink.getA());
  EXPECT_EQ(M_PI_4, prismaticLink.getAlpha());
  EXPECT_EQ(0, prismaticLink.getD());
  EXPECT_EQ(M_PI, prismaticLink.getTheta());
  EXPECT_EQ(kinematics::eJoint::PRISMATIC, prismaticLink.getType());

  EXPECT_ANY_THROW(kinematics::Link(5, 0,0, kinematics::eJoint::STATIC, 0, 20));
}

TEST(Link, CheckConstraints)
{
  kinematics::Link prismaticLink(5, M_PI_4, M_PI, kinematics::eJoint::PRISMATIC,
                                 0, 20);

  EXPECT_EQ(true, prismaticLink.isWithinConstraints(10));
  EXPECT_EQ(false, prismaticLink.isWithinConstraints(60));
  EXPECT_EQ(false, prismaticLink.isWithinConstraints(-10));

  kinematics::Link revoluteLink(2, M_PI_2, 2, kinematics::eJoint::REVOLUTE,
                                -M_PI_2, M_PI_2);

  EXPECT_EQ(true, revoluteLink.isWithinConstraints(M_PI_4));
  EXPECT_EQ(false, revoluteLink.isWithinConstraints(M_PI * 3));
  EXPECT_EQ(false, revoluteLink.isWithinConstraints(M_PI_2 * 5));
}

TEST(Link, ConstrainVariable)
{
  kinematics::Link prismaticLink(5, M_PI_4, M_PI, kinematics::eJoint::PRISMATIC,
                                 0, 20);

  EXPECT_EQ(20, prismaticLink.constrainVariable(50));
  EXPECT_EQ(0, prismaticLink.constrainVariable(-20));
  EXPECT_EQ(10, prismaticLink.constrainVariable(10));

  kinematics::Link revoluteLink(2, M_PI_2, 2, kinematics::eJoint::REVOLUTE,
                                -M_PI_2, M_PI_2);

  EXPECT_EQ(-M_PI_2, revoluteLink.constrainVariable(M_PI * 3));
  EXPECT_EQ(M_PI_2, revoluteLink.constrainVariable(M_PI_2 * 5));
  EXPECT_EQ(-M_PI_2, revoluteLink.constrainVariable(-M_PI_2 * 5));
  EXPECT_EQ(M_PI_4, revoluteLink.constrainVariable(M_PI_4));
}

TEST(Link, TransformationMatrix)
{
  
  // a = 4, alpha = 1, d = 3, theta = 2
  Matrix<double, 4, 4> transformationMatrix{
    { std::cos(2), -sin(2), 0, 4 },
    { std::cos(1) * std::sin(2), cos(1) * cos(2), -sin(1), -3 * sin(1) },
    { sin(1) * sin(2), sin(1) * cos(2), cos(1), 3 * cos(1) },
    { 0, 0, 0, 1 }
  };
  kinematics::Link linkCheck(4, 1, 3, 2, kinematics::eJoint::STATIC);
  EXPECT_EQ(transformationMatrix, linkCheck.transformationMatrix(0));

  // Expect warning due to violating constraints
  kinematics::Link revoluteCheck(4, 1, 3, kinematics::eJoint::REVOLUTE, 0, 1);
  EXPECT_EQ(transformationMatrix, revoluteCheck.transformationMatrix(2));

  kinematics::Link prismaticCheck(4, 1, 2, kinematics::eJoint::PRISMATIC, 0, 10);
  EXPECT_EQ(transformationMatrix, prismaticCheck.transformationMatrix(3));
}