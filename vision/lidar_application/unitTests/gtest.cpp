// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase2)
{
		  EXPECT_EQ(1000, 1000);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
		  testing::InitGoogleTest(&argc, argv);
		  // ros::init(argc, argv, "tester");
		  // ros::NodeHandle nh;
		  return RUN_ALL_TESTS();
}