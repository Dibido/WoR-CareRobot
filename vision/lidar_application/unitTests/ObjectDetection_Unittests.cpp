// Bring in gtest
#include <gtest/gtest.h>

#include <ros/console.h>
#include <ros/ros.h>

#define private public
#include "lidar_application/ObjectDetection.hpp"
#undef private

using namespace lidar_application;

TEST(ObjectDetection, detectObjects_singleObject)
{
  double lMaxDistanceDifference_m = 0.2;

  ObjectDetection lObjectDetection(lMaxDistanceDifference_m);

  // Adding initial scan of area
  LidarData lInitialScanData;
  lInitialScanData.addLidarData(0.0, 1.0);
  lInitialScanData.addLidarData(1.0, 4.0);
  lInitialScanData.addLidarData(2.0, 4.0);
  lInitialScanData.addLidarData(3.0, 4.0);
  lInitialScanData.addLidarData(4.0, 1.0);

  lObjectDetection.mInitialScan = lInitialScanData;
  lObjectDetection.mInitialized = true;

  // New scan data, with distance on angle 1.0 2.0 and 3.0 being closer.
  LidarData lNewScanData;
  lNewScanData.addLidarData(0.0, 1.0);
  lNewScanData.addLidarData(1.0, 2.0);
  lNewScanData.addLidarData(2.0, 2.15);
  lNewScanData.addLidarData(3.0, 2.30);
  lNewScanData.addLidarData(4.0, 1.0);

  lObjectDetection.mMostRecentScan = lNewScanData;

  ASSERT_EQ(static_cast<unsigned int>(0),
            lObjectDetection.mDetectedObjects.size());

  lObjectDetection.detectObjects();

  /* We expect that one object is detected as the differences in distance
   * between angles 1.0, 2.0 and 3.0, are smaller then our defined
   * lMaxDistanceDifference_m of 0.2 */
  ASSERT_EQ(static_cast<unsigned int>(1),
            lObjectDetection.mDetectedObjects.size());
}

TEST(ObjectDetection, detectObjects_multipleObjects)
{
  // Different value compared to scenario 1
  double lMaxDistanceDifference_m = 0.1;

  ObjectDetection lObjectDetection(lMaxDistanceDifference_m);

  // Adding initial scan of area
  LidarData lInitialScanData;
  lInitialScanData.addLidarData(0.0, 1.0);
  lInitialScanData.addLidarData(0.1, 4.0);
  lInitialScanData.addLidarData(0.2, 4.0);
  lInitialScanData.addLidarData(0.3, 4.0);
  lInitialScanData.addLidarData(0.4, 1.0);

  lObjectDetection.mInitialScan = lInitialScanData;
  lObjectDetection.mInitialized = true;

  // New scan data, with distance on angle 1.0 2.0 and 3.0 being closer.
  LidarData lNewScanData;
  lNewScanData.addLidarData(0.0, 1.0);
  lNewScanData.addLidarData(0.1, 2.0);
  lNewScanData.addLidarData(0.2, 2.15);
  lNewScanData.addLidarData(0.3, 2.30);
  lNewScanData.addLidarData(0.4, 1.0);

  lObjectDetection.mMostRecentScan = lNewScanData;

  ASSERT_EQ(static_cast<unsigned int>(0),
            lObjectDetection.mDetectedObjects.size());

  lObjectDetection.detectObjects();

  /** We expect that 3 objects are detected as the differences in distance
   * between angles 0.1, 0.2 and 0.3,
   * are bigger then our defined lMaxDistanceDifference_m of 0.1 */
  EXPECT_EQ(static_cast<unsigned int>(3),
            lObjectDetection.mDetectedObjects.size());
}

TEST(ObjectDetection, convertVectorsTo2D)
{
  ObjectDetection lObjectDetection;

  // North, east, south and north direction again (we start north and rotate
  // clockwards).
  std::vector<std::pair<double, double>> lInputVectors;
  lInputVectors.push_back(std::make_pair(0.0, 2.0));
  lInputVectors.push_back(std::make_pair(M_PI_2, 2.0));
  lInputVectors.push_back(std::make_pair(M_PI, 2.0));
  lInputVectors.push_back(std::make_pair(2 * M_PI, 1.0));

  // This should lead to the following X/Y results:
  std::vector<std::pair<double, double>> lActual2DPoints;
  lActual2DPoints.push_back(std::make_pair(0.0, 2.0));
  lActual2DPoints.push_back(std::make_pair(2.0, 0.0));
  lActual2DPoints.push_back(std::make_pair(0.0, -2.0));
  lActual2DPoints.push_back(std::make_pair(0.0, 1.0));

  std::vector<std::pair<double, double>> lFunction2DPoints =
      lObjectDetection.convertVectorsTo2D(lInputVectors);

  ASSERT_EQ(lActual2DPoints.size(), lFunction2DPoints.size());

  for (size_t lIndex = 0; lIndex < lActual2DPoints.size(); ++lIndex)
  {
    EXPECT_NEAR(lActual2DPoints.at(lIndex).first,
                lFunction2DPoints.at(lIndex).first,
                std::numeric_limits<double>::epsilon());
    EXPECT_NEAR(lActual2DPoints.at(lIndex).second,
                lFunction2DPoints.at(lIndex).second,
                std::numeric_limits<double>::epsilon());
  }
}

TEST(ObjectDetection, getAverageMeasurement)
{
  ObjectDetection lObjectDetection;

  LidarData lLidarData;

  // Add 4 samples, with average angle being 1.5 and average distance being 50.0
  lLidarData.addLidarData(0.0, 50.0);
  lLidarData.addLidarData(1.0, 50.0);
  lLidarData.addLidarData(2.0, 50.0);
  lLidarData.addLidarData(3.0, 50.0);

  double lAverageAngle = 1.5;
  double lAverageDistance_m = 50.0;

  std::pair<double, double> lAverages =
      lObjectDetection.getAverageMeasurement(lLidarData);

  EXPECT_NEAR(lAverageAngle, lAverages.first,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(lAverageDistance_m, lAverages.second,
              std::numeric_limits<double>::epsilon());
}