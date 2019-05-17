// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite2, testCase3)
{
		  EXPECT_EQ(0,0);
}

// Declare another test
TEST(TestSuite2, testCase4)
{
		  EXPECT_EQ(1000, 0);
}