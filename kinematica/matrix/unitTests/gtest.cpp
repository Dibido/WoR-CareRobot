// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include "matrix/Matrix.hpp"
#include <gtest/gtest.h>

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
