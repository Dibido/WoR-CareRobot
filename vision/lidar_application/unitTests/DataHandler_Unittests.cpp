// Bring in gtest
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#define private public
#include "lidar_application/DataHandler.hpp"
#undef private

using namespace lidar_application;
using ::testing::_;

TEST(DataHandler, ConstructorTopicNames)
{
  std::string lReceiveTopicName = "/receive";
  std::string lPublishTopicName = "/publish";

  DataHandler lDataHandler(lReceiveTopicName, lPublishTopicName);

  EXPECT_EQ(lReceiveTopicName, lDataHandler.mLidarSubscriber.getTopic());
  EXPECT_EQ(lPublishTopicName, lDataHandler.mObjectPublisher.getTopic());
}

TEST(DataHandler, IsNewDataAvailable)
{
  DataHandler lDataHandler;

  // We expect the default to be false
  EXPECT_FALSE(lDataHandler.isNewDataAvailable());

  lDataHandler.mNewDataAvailable = true;

  EXPECT_TRUE(lDataHandler.isNewDataAvailable());
}

TEST(DataHandler, GetNewData)
{
  DataHandler lDataHandler;

  LidarData lLidarData;
  lLidarData.mMeasurements.insert(std::pair<double, double>(1.0, 1.0));

  lDataHandler.mLidarData = lLidarData;
  lDataHandler.mNewDataAvailable = true;

  // New data available stays true
  EXPECT_TRUE(lDataHandler.isNewDataAvailable());
  EXPECT_TRUE(lDataHandler.isNewDataAvailable());

  // After receiving data
  EXPECT_EQ(lLidarData.mMeasurements.size(),
            lDataHandler.getLidarData().mMeasurements.size());

  // There should be no 'new' data available
  EXPECT_FALSE(lDataHandler.isNewDataAvailable());
}
