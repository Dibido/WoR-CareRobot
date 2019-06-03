// Bring in gtest
#include <gtest/gtest.h>
#include "controller/TrajectoryProvider.hpp"
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
