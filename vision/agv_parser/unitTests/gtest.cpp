// Bring in gtest
#include <gtest/gtest.h>

#include <ros/ros.h>

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "AgvParser_Tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}