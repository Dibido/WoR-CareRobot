#include "kinematics/Link.hpp"
#include <gtest/gtest.h>
namespace kinematics
{
  TEST(Link, Constructors)
  {
    Link link0(1, M_PI, 3, 0, eJoint::STATIC);

    EXPECT_EQ(1, link0.getA());
    EXPECT_EQ(M_PI, link0.getAlpha());
    EXPECT_EQ(3, link0.getD());
    EXPECT_EQ(0, link0.getTheta());
    EXPECT_EQ(eJoint::STATIC, link0.getType());

    Link revoluteLink(2, M_PI_2, 2, eJoint::REVOLUTE, 0, 4);

    EXPECT_EQ(2, revoluteLink.getA());
    EXPECT_EQ(M_PI_2, revoluteLink.getAlpha());
    EXPECT_EQ(2, revoluteLink.getD());
    EXPECT_EQ(0, revoluteLink.getTheta());
    EXPECT_EQ(eJoint::REVOLUTE, revoluteLink.getType());

    Link prismaticLink(5, M_PI_4, M_PI, eJoint::PRISMATIC, 0, 20);

    EXPECT_EQ(5, prismaticLink.getA());
    EXPECT_EQ(M_PI_4, prismaticLink.getAlpha());
    EXPECT_EQ(0, prismaticLink.getD());
    EXPECT_EQ(M_PI, prismaticLink.getTheta());
    EXPECT_EQ(eJoint::PRISMATIC, prismaticLink.getType());

    EXPECT_ANY_THROW(Link(5, 0, 0, eJoint::STATIC, 0, 20));
  }

  TEST(Link, CheckConstraints)
  {
    Link prismaticLink(5, M_PI_4, M_PI, eJoint::PRISMATIC, 0, 20);

    EXPECT_EQ(true, prismaticLink.isWithinConstraints(10));
    EXPECT_EQ(false, prismaticLink.isWithinConstraints(60));
    EXPECT_EQ(false, prismaticLink.isWithinConstraints(-10));

    Link revoluteLink(2, M_PI_2, 2, eJoint::REVOLUTE, -M_PI_2, M_PI_2);

    EXPECT_EQ(true, revoluteLink.isWithinConstraints(M_PI_4));
    EXPECT_EQ(false, revoluteLink.isWithinConstraints(M_PI * 3));
    EXPECT_EQ(false, revoluteLink.isWithinConstraints(M_PI_2 * 5));
  }

  TEST(Link, ConstrainVariable)
  {
    Link prismaticLink(5, M_PI_4, M_PI, eJoint::PRISMATIC, 0, 20);

    EXPECT_EQ(20, prismaticLink.constrainVariable(50));
    EXPECT_EQ(0, prismaticLink.constrainVariable(-20));
    EXPECT_EQ(10, prismaticLink.constrainVariable(10));

    Link revoluteLink(2, M_PI_2, 2, eJoint::REVOLUTE, -M_PI_2, M_PI_2);

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
    Link linkCheck(4, 1, 3, 2, eJoint::STATIC);
    EXPECT_EQ(transformationMatrix, linkCheck.transformationMatrix(0));

    // Expect warning due to violating constraints
    Link revoluteCheck(4, 1, 3, eJoint::REVOLUTE, 0, 1);
    EXPECT_EQ(transformationMatrix, revoluteCheck.transformationMatrix(2));

    Link prismaticCheck(4, 1, 2, eJoint::PRISMATIC, 0, 10);
    EXPECT_EQ(transformationMatrix, prismaticCheck.transformationMatrix(3));
  }
} // namespace kinematics