// Bring in gtest
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>


#define private public
#include "LidarData.hpp"
#include "DataHandler.hpp"
#undef private

using namespace lidar_application;

TEST(LidarData, Constructor_UnequalVectorSizes)
{
  std::vector<double> lAngles = { 0, 0 };
  std::vector<double> lDistances_m = { 0, 0 };

  EXPECT_EQ(lAngles.size(), lDistances_m.size());

  EXPECT_NO_THROW(LidarData(lAngles, lDistances_m));

  lDistances_m.push_back(0);

  EXPECT_THROW(LidarData(lAngles, lDistances_m), std::logic_error);
}

TEST(LidarData, AddLidarData_MultipleSamples)
{
  std::vector<double> lAngles = { 0, 0 };
  std::vector<double> lDistances_m = { 0, 0 };

  LidarData lLidarData(lAngles, lDistances_m);

  EXPECT_NO_THROW(lLidarData.addLidarData(lAngles, lDistances_m));

  lDistances_m.push_back(0);

  EXPECT_THROW(lLidarData.addLidarData(lAngles, lDistances_m),
               std::logic_error);
}

TEST(LidarData, AddLidarData_SingleSample)
{
  LidarData lLidarData;

  double lAngle = 0.15;
  double lDistance_m = 1.00;

  lLidarData.addLidarData(lAngle, lDistance_m);

  EXPECT_EQ(lAngle, lLidarData.mAngles.at(0));
  EXPECT_EQ(lDistance_m, lLidarData.mDistances_m.at(0));
}

TEST(LidarData, ResetEmptyVectors)
{
  std::vector<double> lAngles = { 0, 0 };
  std::vector<double> lDistances_m = { 0, 0 };

  LidarData lLidarData(lAngles, lDistances_m);

  EXPECT_EQ(lAngles.size(), lLidarData.mAngles.size());
  EXPECT_EQ(lDistances_m.size(), lLidarData.mDistances_m.size());

  lLidarData.reset();

  EXPECT_EQ(static_cast<unsigned int>(0), lLidarData.mAngles.size());
  EXPECT_EQ(static_cast<unsigned int>(0), lLidarData.mDistances_m.size());
}