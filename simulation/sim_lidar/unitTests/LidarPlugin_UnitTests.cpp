// Bring in gtest
#include <gtest/gtest.h>
// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
// Include messages
#include <sensor_interfaces/LidarData.h>
#include <sensor_msgs/LaserScan.h>
// Make everything public in to test code.
#define private public
#include <sim_lidar/LidarPlugin.hpp>
#undef private

#include <string.h>

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
  const double lMaximumDeviation = 0.00005;
  // Set up message
  sensor_msgs::LaserScan lLaserScanMessage;
  lLaserScanMessage.angle_min = static_cast<float>(-M_PI);
  lLaserScanMessage.angle_max = static_cast<float>(M_PI);
  lLaserScanMessage.angle_increment =
      (lLaserScanMessage.angle_max - lLaserScanMessage.angle_min) / lNumberOfMeasurements;
  lLaserScanMessage.range_min = static_cast<float>(0.1);
  lLaserScanMessage.range_max = 3.0;
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
  ASSERT_EQ(lLaserScanMessage.ranges.size(), lLidarMessage.distances.size());
  // Check conversion
  EXPECT_EQ(lLaserScanMessage.header.frame_id, lLidarMessage.header.frame_id);
  EXPECT_EQ(lLaserScanMessage.angle_increment, lLidarMessage.measurement_angles.at(0));
  EXPECT_NEAR((lLaserScanMessage.angle_increment * lNumberOfMeasurements),
              lLidarMessage.measurement_angles.at(lLidarMessage.measurement_angles.size() - 1), lMaximumDeviation);
  EXPECT_EQ(lLaserScanMessage.ranges, lLidarMessage.distances);
  float lExpectedAngle = 0.0;
  for (unsigned int i = 0; i < lLidarMessage.distances.size(); i++)
  {
    lExpectedAngle += lLaserScanMessage.angle_increment;
    EXPECT_EQ(lExpectedAngle, lLidarMessage.measurement_angles.at(i));
  }
}

/**
 * @brief Test the protocol ScanDataConversion
 * Check whether the data that is sent to the ScanData topic is converted
 * properly.
 */
/*
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
  sensor_msgs::LaserScan lLaserScanDataMessage;
  lLaserScanDataMessage = lLidarPlugin.convertToLaserScan(lLaserScanMessage);
  // Check conversion
  // TODO
}
*/