// Bring in gtest
#include <gtest/gtest.h>

#define private public
#include "../include/LidarData.hpp"
#include "../include/ObjectDetection.hpp"
#undef private

namespace LidarUnittests
{
  void initRos()
  {
    std::vector<std::pair<std::string, std::string>> lRemappings;
    ros::init(lRemappings, "ObjectDetection");
  }
}; // namespace LidarUnittests

TEST(LidarData, ConstructorVectorSizes)
{
  LidarUnittests::initRos();

  std::vector<double> lAngles = {0, 0};
  std::vector<double> lDistances_m = {0, 0};

  EXPECT_EQ(lAngles.size(),
            lDistances_m.size());
  
  EXPECT_NO_THROW(LidarData(lAngles, lDistances_m));

  lDistances_m.push_back(0);

  EXPECT_THROW(LidarData(lAngles, lDistances_m), std::logic_error);
}

TEST(LidarData, AddSamples)
{
  std::vector<double> lAngles = {0, 0};
  std::vector<double> lDistances_m = {0, 0};

  LidarData lLidarData(lAngles, lDistances_m);

  EXPECT_NO_THROW(lLidarData.addLidarData(lAngles, lDistances_m));
  
  lDistances_m.push_back(0);

  EXPECT_THROW(lLidarData.addLidarData(lAngles, lDistances_m), std::logic_error);
}

TEST(LidarData, Reset)
{
  std::vector<double> lAngles = {0, 0};
  std::vector<double> lDistances_m = {0, 0};

  LidarData lLidarData(lAngles, lDistances_m);

  EXPECT_EQ(lAngles.size(), lLidarData.mAngles.size());
  EXPECT_EQ(lDistances_m.size(), lLidarData.mDistances_m.size());

  lLidarData.reset();

  EXPECT_EQ(static_cast<unsigned int>(0), lLidarData.mAngles.size());
  EXPECT_EQ(static_cast<unsigned int>(0), lLidarData.mDistances_m.size());
}