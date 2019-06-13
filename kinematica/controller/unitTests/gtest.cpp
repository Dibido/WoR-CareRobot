// Bring in gtest
#include "controller/TrajectoryProvider.hpp"
#include <gtest/gtest.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Controller_UnitTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
