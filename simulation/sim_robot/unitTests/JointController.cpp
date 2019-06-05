#include <ros/ros.h>

#define private public
#include <sim_robot/JointController.hpp>
#undef private
// Include messages
// Bring in gtest
#include <gtest/gtest.h>

TEST(JointController, converseScaleToRad)
{
  gazebo::physics::JointPtr joint;
  const std::string mName = "mName";
  const jointChannel_t mChannel = 0;
  const jointPw_t mMinPw = 0;
  const jointPw_t mMaxPw = 0; 
  const jointRad_t mMinRad = -10;
  const jointRad_t mMaxRad = 10;
  const jointRad_t mMediumRad = mMinRad + (abs(mMinRad-mMaxRad)/2.0);
  const jointVel_t mMaxVel = 0;

  gazebo::JointController jointController(joint, mName, mChannel, mMinPw, mMaxPw, mMinRad, mMaxRad,
                                           mMaxVel);
  
  EXPECT_EQ(jointController.converseScaleToRad(0, 0, 10), mMinRad);
  EXPECT_EQ(jointController.converseScaleToRad(10, 0, 10), mMaxRad);
  EXPECT_EQ(jointController.converseScaleToRad(5, 0, 10), mMediumRad);
  EXPECT_EQ(jointController.converseScaleToRad(10, 10, 0), mMaxRad);
  EXPECT_EQ(jointController.converseScaleToRad(0, 10, 0), mMinRad);
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
