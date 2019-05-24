// Bring in gtest
#include <gtest/gtest.h>

#define private public
#include "DataHandler.hpp"
#undef private

using namespace lidar_application;

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
  lLidarData.mAngles.push_back(1);
  lLidarData.mDistances_m.push_back(1);

  lDataHandler.mLidarData = lLidarData;
  lDataHandler.mNewDataAvailable = true;

  // New data available stays true
  EXPECT_TRUE(lDataHandler.isNewDataAvailable());
  EXPECT_TRUE(lDataHandler.isNewDataAvailable());

  // After receiving data 
  EXPECT_EQ(lLidarData.mAngles.size(), lDataHandler.getLidarData().mAngles.size());

  // There should be no 'new' data available
  EXPECT_FALSE(lDataHandler.isNewDataAvailable());
}