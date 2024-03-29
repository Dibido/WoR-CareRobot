#include <ros/ros.h>

#define private public
#include <sim_robot/JointController.hpp>
#undef private
// Include messages
// Bring in gtest
#include <gtest/gtest.h>

TEST(JointController, convertScaleToRad)
{
  gazebo::physics::JointPtr lJoint;
  const std::string cName = "mName";
  const jointChannel_t cChannel = 0;
  const jointPw_t cMinPw = 0;
  const jointPw_t cMaxPw = 0;
  const jointRad_t cMinRad = -10;
  const jointRad_t cMaxRad = 10;
  const jointRad_t cMediumRad = cMinRad + (abs(cMinRad - cMaxRad) / 2.0);
  const jointVel_t cMaxVel = 0;

  const gazebo::JointController cJointController(
      lJoint, cName, cChannel, cMinPw, cMaxPw, cMinRad, cMaxRad, cMaxVel);

  EXPECT_NO_THROW(cJointController.convertScaleToRad(5, 0, 10));
  ASSERT_THROW(cJointController.convertScaleToRad(-5, 0, 10),
               std::invalid_argument);
  ASSERT_THROW(cJointController.convertScaleToRad(15, 0, 10),
               std::invalid_argument);
  EXPECT_EQ(cJointController.convertScaleToRad(0, 0, 10), cMinRad);
  EXPECT_EQ(cJointController.convertScaleToRad(10, 0, 10), cMaxRad);
  EXPECT_EQ(cJointController.convertScaleToRad(5, 0, 10), cMediumRad);
  EXPECT_EQ(cJointController.convertScaleToRad(10, 10, 0), cMinRad);
  EXPECT_EQ(cJointController.convertScaleToRad(0, 10, 0), cMaxRad);
  EXPECT_EQ(cJointController.convertScaleToRad(-10, -10, 0), cMinRad);
}

TEST(JointController, isInRange)
{
  gazebo::physics::JointPtr lJoint;
  const gazebo::JointController cJointController(lJoint, "mName", 0, -10, 10,
                                                 -10, 10, 10);
  EXPECT_EQ(cJointController.isInRange(5, 0, 10), true);
  EXPECT_EQ(cJointController.isInRange(-5, 0, 10), false);
  EXPECT_EQ(cJointController.isInRange(15, 0, 10), false);
}

TEST(operatorJointController, equalsOperator)
{
  gazebo::physics::JointPtr joint;

  gazebo::JointController jointController1(joint, "mName", 0, -10, 10, -10, 10,
                                           10);
  gazebo::JointController jointController2(joint, "mName", 0, -10, 10, -10, 10,
                                           10);

  EXPECT_EQ(jointController1, jointController2);
}
TEST(operatorJointController, equalsNotOperator)
{
  gazebo::physics::JointPtr joint;

  gazebo::JointController jointController1(joint, "mName1", 0, -10, 10, -10, 10,
                                           10);
  gazebo::JointController jointController2(joint, "mName2", 0, -10, 10, -10, 10,
                                           10);

  EXPECT_NE(jointController1, jointController2);
}

TEST(operatorJointController, assigmentOperator)
{
  gazebo::physics::JointPtr joint;

  gazebo::JointController jointController1(joint, "mName1", 0, -10, 10, -10, 10,
                                           10);
  gazebo::JointController jointController2(joint, "mName2", 0, -10, 10, -10, 10,
                                           10);
  jointController2 = jointController1;

  EXPECT_EQ(jointController1, jointController2);
}

TEST(moveJointController, convertDegrees2Radians)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, 500, 2500, -M_PI,
                                           M_PI, 180);
  EXPECT_EQ((180 * M_PI) / 180, jointController1.convertDegrees2Radians(180));
}
TEST(moveJointController, convertPw2Radians)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, 500, 2500, -M_PI,
                                           M_PI, 180);
  EXPECT_EQ(M_PI, jointController1.convertPw2Radians(2500));
  EXPECT_EQ(-M_PI, jointController1.convertPw2Radians(500));
}
TEST(moveJointController, moveTheta180)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, 500, 2500, -M_PI,
                                           M_PI, 180);

  EXPECT_EQ(true, jointController1.moveTheta(1, 1, 0, 30));
  EXPECT_EQ(180, jointController1.getMaxVel());
  EXPECT_EQ(M_PI, jointController1.getCurrentVel());
  jointController1.moveTheta(1, 0.1, 0, 30);
  EXPECT_EQ((M_PI / 10), jointController1.getCurrentVel());
}

TEST(moveJointController, moveTheta150)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, 500, 2500, -M_PI,
                                           M_PI, 150);
  EXPECT_EQ(150, jointController1.getMaxVel());
  EXPECT_EQ(true, jointController1.moveTheta(1, 1, 0, 30));
  EXPECT_EQ((150 * M_PI) / 180, jointController1.getCurrentVel());
  jointController1.moveTheta(1, 0.1, 0, 30);
  EXPECT_EQ(((150 * M_PI / 180) / 10), jointController1.getCurrentVel());
}
TEST(moveJointController, move)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, 500, 2500, -3, 3,
                                           2);
  EXPECT_EQ(true, jointController1.move(2500, 1, 0, 30));
  EXPECT_EQ(3, jointController1.getTargetPos());
  EXPECT_EQ(false, jointController1.move(2700, 1, 0, 30));
}

TEST(moveJointController, stop)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, -1, 1, -1, 1, 2);

  jointController1.stop();

  EXPECT_EQ(0, jointController1.getCurrentVel());
}
TEST(updateJointController, update)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, -1, 1, -1, 1, 2);
  jointController1.update();

  EXPECT_EQ(0, jointController1.getCurrentVel());
  EXPECT_EQ(0, jointController1.getCurrentPos());
}

TEST(equalsDoubleJointController, equalsDouble)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, -1, 1, -1, 1, 2);

  EXPECT_EQ(true, gazebo::equalsDouble(1.0, 1.0));
  EXPECT_EQ(false, gazebo::equalsDouble(2.0, 1.0));
  EXPECT_EQ(false, gazebo::equalsDouble(1.0, 2.0));
}
