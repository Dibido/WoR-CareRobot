// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>

#include <gtest/gtest.h>

// Declare a test

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");

  return RUN_ALL_TESTS();
}
