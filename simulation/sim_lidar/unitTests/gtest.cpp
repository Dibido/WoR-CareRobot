// Bring in my package's API, which is what I'm testing
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <ros/ros.h>
// Include messages
#include <sensor_interfaces/LidarData.h>
#include <sensor_msgs/LaserScan.h>
// Bring in gtest
#include <gtest/gtest.h>
#include <iostream>
#include <string.h>

#include <type_traits>
// Make everything public in to test code.
#define private public
#include <sim_lidar/LidarPlugin.hpp>
#undef private

/**
 * @brief Test the protocol LidarDataConversion
 * Check whether the data that is sent to the lidar plugin is converted
 * properly.
 */
TEST(LidarData, LidarDataConversion)
{
  // Set values
  gazebo::LidarPlugin lLidarPlugin;
  const unsigned int lNumberOfMeasurements = 360;
  // Set up message
  // ConstLaserScanStampedPtr
  sensor_msgs::LaserScan lLaserScanMessage;
  lLaserScanMessage.time_increment = 2;
  lLaserScanMessage.angle_min = static_cast<float>(-M_PI);
  lLaserScanMessage.angle_max = static_cast<float>(M_PI);
  lLaserScanMessage.range_min = static_cast<float>(0.1);
  lLaserScanMessage.range_max = 3;
  lLaserScanMessage.ranges.clear();
  lLaserScanMessage.intensities.clear();
  for (unsigned int i = 0; i < lNumberOfMeasurements; i++)
  {
    lLaserScanMessage.ranges.push_back(2);
    lLaserScanMessage.intensities.push_back(0);
  }
  // Call parser function
  sensor_interfaces::LidarData lLidarMessage;
  lLidarMessage = lLidarPlugin.convertToLidarData(lLaserScanMessage);
  // Check size
  ASSERT_EQ(lLidarMessage.distances.size(), lLaserScanMessage.ranges.size());
  // Check conversion
  EXPECT_EQ(lLidarMessage.header.frame_id, lLaserScanMessage.header.frame_id);
  EXPECT_EQ(lLidarMessage.measurement_angles.at(0), 0.0);
  EXPECT_EQ(lLidarMessage.measurement_angles.at(
                lLidarMessage.measurement_angles.size()),
            (M_PI * 2));
  EXPECT_EQ(lLidarMessage.distances, lLaserScanMessage.ranges);
  float lExpectedAngle = 0;
  for (unsigned int i = 0; i < lLidarMessage.distances.size(); i++)
  {
    lExpectedAngle += lLaserScanMessage.angle_increment;
    EXPECT_EQ(lLidarMessage.measurement_angles.at(i), lExpectedAngle);
  }
}

/**
 * @brief Test the protocol ScanDataConversion
 * Check whether the data that is sent to the ScanData topic is converted
 * properly.
 */
TEST(LidarData, ScanDataConversion)
{
  // Set values
  gazebo::LidarPlugin lLidarPlugin;
  const unsigned int lNumberOfMeasurements = 360;
  // Set up message
  sensor_msgs::LaserScan lLaserScanMessage;
  lLaserScanMessage.time_increment = 2;
  lLaserScanMessage.angle_min = static_cast<float>(-M_PI);
  lLaserScanMessage.angle_max = static_cast<float>(M_PI);
  lLaserScanMessage.range_min = static_cast<float>(0.1);
  lLaserScanMessage.range_max = 3;
  lLaserScanMessage.ranges.clear();
  lLaserScanMessage.intensities.clear();
  for (unsigned int i = 0; i < lNumberOfMeasurements; i++)
  {
    lLaserScanMessage.ranges.push_back(2);
    lLaserScanMessage.intensities.push_back(0);
  }
  // Call parser function
  gazebo_msgs::LaserScan lLaserScanDataMessage;
  lLaserScanDataMessage = lLidarPlugin.convertToLaserScan(lLaserScanMessage);
  // Check conversion
  // TODO
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LidarData_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}