// Bring in gtest
#include <gtest/gtest.h>

#include <ros/ros.h>

#define private public
#include "lidar_application/DataHandler.hpp"
#include "lidar_application/LidarData.hpp"
#undef private

using namespace lidar_application;

TEST(LidarData, Constructors)
{
  LidarData lLidarData;
  EXPECT_EQ(0, static_cast<int>(lLidarData.mMeasurements.size()));

  std::map<double, double> lInputMap;
  lInputMap.insert(std::pair<double, double>(0.0, 0.0));
  lInputMap.insert(std::pair<double, double>(0.1, 0.0));

  LidarData lLidarDataWithMap(lInputMap);

  EXPECT_EQ(lInputMap.size(), lLidarDataWithMap.mMeasurements.size());

  EXPECT_EQ(lInputMap.begin()->first,
            lLidarDataWithMap.mMeasurements.begin()->first);
  EXPECT_EQ(lInputMap.begin()->second,
            lLidarDataWithMap.mMeasurements.begin()->second);
}

TEST(LidarData, AddLidarData_MultipleSamplesUnequalVectorSize)
{
  std::vector<double> lAngles = { 0.0, 0.0 };
  std::vector<double> lDistances_m = { 0.0, 0.0 };

  LidarData lLidarData;

  EXPECT_NO_THROW(lLidarData.addLidarData(lAngles, lDistances_m));

  lDistances_m.push_back(0.0);

  // Unequal vector size, should throw error
  EXPECT_THROW(lLidarData.addLidarData(lAngles, lDistances_m),
               std::logic_error);
}

TEST(LidarData, AddLidarData_AngleOutOfRange)
{
  LidarData lLidarData;

  const double lAngle = 0.0;
  const double lDistance_m = 1.0;

  EXPECT_NO_THROW(lLidarData.addLidarData(lAngle, lDistance_m));

  EXPECT_NEAR(lAngle, lLidarData.mMeasurements.begin()->first,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(lDistance_m, lLidarData.mMeasurements.begin()->second,
              std::numeric_limits<double>::epsilon());

  // Negative angles shouldn't be accepted
  EXPECT_THROW(lLidarData.addLidarData(-0.1, 0.0), std::range_error);

  // Angles greater then 2 * M_PI radians (360 degrees) should also raise an
  // error
  EXPECT_THROW(lLidarData.addLidarData((2 * M_PI) + 1.0, 1.0),
               std::range_error);
}

TEST(LidarData, AddLidarData_Map)
{
  LidarData lLidarData;

  EXPECT_EQ(0, static_cast<int>(lLidarData.mMeasurements.size()));

  std::map<double, double> lInputMap;
  lInputMap.insert(std::pair<double, double>(0.0, 0.0));
  lInputMap.insert(std::pair<double, double>(0.1, 0.0));

  lLidarData.addLidarData(lInputMap);

  EXPECT_NEAR(lInputMap.begin()->first, lLidarData.mMeasurements.begin()->first,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(lInputMap.begin()->second,
              lLidarData.mMeasurements.begin()->second,
              std::numeric_limits<double>::epsilon());
}

TEST(LidarData, Reset)
{
  std::map<double, double> lInputMap;
  lInputMap.insert(std::pair<double, double>(0.0, 0.0));
  lInputMap.insert(std::pair<double, double>(0.1, 0.0));

  LidarData lLidarData(lInputMap);

  EXPECT_EQ(lInputMap.size(), lLidarData.mMeasurements.size());

  lLidarData.reset();

  EXPECT_EQ(static_cast<unsigned int>(0),
            static_cast<unsigned int>(lLidarData.mMeasurements.size()));
}