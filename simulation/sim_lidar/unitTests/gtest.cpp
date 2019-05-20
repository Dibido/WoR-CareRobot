// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
#include <sim_lidar/LidarPlugin.h>
// Include messages
#include <sensor_msgs/LaserScan.h>
#include <sensor_interfaces/LidarData.hpp>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestProtocol, TestLidarData)
{
  LidarPlugin lLidarPLugin;
  // Publish data to the sensor topic
  ros::Publisher lDataPublisher = nh.advertise("/sensor/lidar/scan", 1);
  sensor_msgs::LaserScan lLaserScanMessage;
  lLaserScanMessage.
  lDataPublisher.publish(lLaserScanMessage);
  // Read the output topic
  ros::Subscriber lDataSubscriber = nh.subscribe("/sensor/lidar", 1);
  gazeboSub = nh->Subscribe(parentSensor->Topic(), &LidarPlugin::OnScan, this);
  // Check the size
  EXPECT_EQ();
  // Compare the values
  EXPECT_EQ(1000, 1000);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
		  testing::InitGoogleTest(&argc, argv);
		  ros::init(argc, argv, "tester");
		  ros::NodeHandle nh;
		  return RUN_ALL_TESTS();
}
