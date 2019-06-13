#include "kinematics/EulerAngles.hpp"
#include <gtest/gtest.h>
namespace kinematics
{
  TEST(EulerAngles, ConstructorConvert)
  {

    double yaw = M_PI;
    double pitch = M_PI_4;
    double roll = M_PI_2;

    EulerAngles euler(yaw, pitch, roll);

    EXPECT_EQ(yaw, euler.mYaw_rad);
    EXPECT_EQ(pitch, euler.mPitch_rad);
    EXPECT_EQ(roll, euler.mRoll_rad);

    Matrix<double, 3, 3> rotationMatrix = euler.toRotationMatrix();

    EulerAngles euler2(rotationMatrix);

    EXPECT_EQ(yaw, euler2.mYaw_rad);
    EXPECT_EQ(pitch, euler2.mPitch_rad);
    EXPECT_EQ(roll, euler2.mRoll_rad);
  }

} // namespace kinematics