#include <ros/ros.h>
#include <sim_robot/JointController.hpp>

// Include messages
// Bring in gtest
#include <gtest/gtest.h>

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

TEST(moveJointController, moveTheta)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, -1, 1, -1, 1, 2);

  EXPECT_EQ(true, jointController1.moveTheta(1, 1, 0, 30));
  // 1.57079632679489661923 = M_PI_2 * 1;
  EXPECT_EQ(1.57079632679489661923, jointController1.getCurrentVel());
  jointController1.moveTheta(1, 10, 0, 30);
  EXPECT_EQ(2, jointController1.getCurrentVel());
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

// TEST(updateJointController, run)
// {
//   gazebo::physics::JointPtr joint;
//   gazebo::JointController jointController1(joint, "mName1", 0, -1, 1, -1, 1, 2);
//   jointController1.setCurrentPos;

//   EXPECT_EQ(0, jointController1.getCurrentVel());
//   EXPECT_EQ(0, jointController1.getCurrentPos());
// }

TEST(equalsDoubleJointController, equalsDouble)
{
  gazebo::physics::JointPtr joint;
  gazebo::JointController jointController1(joint, "mName1", 0, -1, 1, -1, 1, 2);

  EXPECT_EQ(true, gazebo::equalsDouble(1.0, 1.0));
  EXPECT_EQ(false, gazebo::equalsDouble(2.0, 1.0));
  EXPECT_EQ(false, gazebo::equalsDouble(1.0, 2.0));
}
