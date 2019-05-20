// Bring in my package's API, which is what I'm testing
/* #include "../include/Demo.h" */
// Bring in gtest
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
		  testing::InitGoogleTest(&argc, argv);
		  // ros::init(argc, argv, "tester");
		  // ros::NodeHandle nh;
		  return RUN_ALL_TESTS();
}
