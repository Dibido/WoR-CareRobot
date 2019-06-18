// Bring in gtest
#include <gtest/gtest.h>

#include <ros/console.h>
#include <ros/ros.h>

#define private public
#include "lidar_application/ObjectDetection.hpp"
#undef private

using namespace lidar_application;

namespace objectdetection_unittest_constants
{
  // mMaxDistanceDifference
  const double cMaxDifference_m = 0.20;

  // 3 has been proven a good default, see ObjectDetection.hpp
  const unsigned int cMinNumberAdjacentAngles = 3;

  // 10 has proven to be a good default, 10 cycles of the lidar takes about a
  // second see ObjectDetection.hpp
  const unsigned int cNumberOfInitialScanRounds = 10;
} // namespace objectdetection_unittest_constants

TEST(ObjectDetection, detectObjects_singleObject)
{
  double lMaxDistanceDifference_m = 0.2;

  ObjectDetection lObjectDetection(
      lMaxDistanceDifference_m, true,
      objectdetection_unittest_constants::cMinNumberAdjacentAngles,
      objectdetection_unittest_constants::cNumberOfInitialScanRounds);

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
  lInitialScanData.addLidarData(0.0, 1.00);
  lInitialScanData.addLidarData(0.1, 4.01);
  lInitialScanData.addLidarData(0.2, 4.02);
  lInitialScanData.addLidarData(0.3, 4.03);
  lInitialScanData.addLidarData(0.4, 1.04);

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
   * between angles 0.1, 0.2 and 0.3, and their neighbours
   * are bigger then our defined lMaxDistanceDifference_m of 0.1 */
  EXPECT_EQ(static_cast<unsigned int>(3),
            lObjectDetection.mDetectedObjects.size());
}

TEST(ObjectDetection, ignoreSmallObjects)
{
  const double lMaxDifference_m = 0.20;
  const unsigned int lMinNumberAdjacentAngles = 3;

  bool lIgnoreSmallObjects = false;
  ObjectDetection lObjectDetectionAllObjects(
      lMaxDifference_m, lIgnoreSmallObjects, lMinNumberAdjacentAngles,
      objectdetection_unittest_constants::cNumberOfInitialScanRounds);

  lIgnoreSmallObjects = true;
  ObjectDetection lObjectDetectionIgnoreSmallObjects(
      lMaxDifference_m, lIgnoreSmallObjects, lMinNumberAdjacentAngles,
      objectdetection_unittest_constants::cNumberOfInitialScanRounds);

  std::map<double, double> lInitialSamples;
  lInitialSamples.insert(std::pair<double, double>(0.0, 1.0));
  lInitialSamples.insert(std::pair<double, double>(0.1, 1.0));
  lInitialSamples.insert(std::pair<double, double>(0.2, 1.0));
  lInitialSamples.insert(std::pair<double, double>(0.3, 1.0));
  lInitialSamples.insert(std::pair<double, double>(0.4, 1.0));

  lObjectDetectionAllObjects.mInitialScan.addLidarData(lInitialSamples);
  lObjectDetectionIgnoreSmallObjects.mInitialScan.addLidarData(lInitialSamples);

  std::map<double, double> lRecentScan;
  lRecentScan.insert(std::pair<double, double>(0.05, 1.5));
  lRecentScan.insert(std::pair<double, double>(0.15, 1.0));
  lRecentScan.insert(std::pair<double, double>(0.25, 1.5));
  lRecentScan.insert(std::pair<double, double>(0.35, 1.0));

  lObjectDetectionAllObjects.mMostRecentScan.addLidarData(lRecentScan);
  lObjectDetectionIgnoreSmallObjects.mMostRecentScan.addLidarData(lRecentScan);

  // The angles 0.05 and 0.25 conflict with the initial scandata, so we expect 2
  // objects to be detected.
  unsigned int lExpectedNumberOfObjects = 2;

  lObjectDetectionAllObjects.detectObjects();

  unsigned int lNumberOfDetectedObjects = static_cast<unsigned int>(
      lObjectDetectionAllObjects.mDetectedObjects.size());

  EXPECT_EQ(lExpectedNumberOfObjects, lNumberOfDetectedObjects);

  /** Now if objectdetection ignores small objects, we expect 0 objects to be
   * detected. This is because only 2 single angles conflict, not 3 adjacent
   * ones as defined in lMinNumberOfAdjacentAngles */
  lExpectedNumberOfObjects = 0;

  lObjectDetectionIgnoreSmallObjects.detectObjects();

  lNumberOfDetectedObjects = static_cast<unsigned int>(
      lObjectDetectionIgnoreSmallObjects.mDetectedObjects.size());

  EXPECT_EQ(lExpectedNumberOfObjects, lNumberOfDetectedObjects);
}

TEST(ObjectDetection, getSurroundingDistances_Exceptions)
{
  ObjectDetection lObjectDetection(
      objectdetection_unittest_constants::cMaxDifference_m, true,
      objectdetection_unittest_constants::cMinNumberAdjacentAngles,
      objectdetection_unittest_constants::cNumberOfInitialScanRounds);

  ASSERT_EQ(
      static_cast<int>(0),
      static_cast<int>(lObjectDetection.mInitialScan.mMeasurements.size()));

  // We expect an exception as mInitialScan contains no data.
  EXPECT_THROW(lObjectDetection.getSurroundingDistances(0.0), std::logic_error);

  lObjectDetection.mInitialScan.addLidarData(0.0, 4.01);
  lObjectDetection.mInitialScan.addLidarData(0.1, 4.01);

  // Angle greater then 2 * M_PI
  const double lAngle = 2 * M_PI + 1;

  EXPECT_THROW(lObjectDetection.getSurroundingDistances(lAngle),
               std::range_error);
}

TEST(ObjectDetection, getSurroundingDistances_Default)
{
  ObjectDetection lObjectDetection(
      objectdetection_unittest_constants::cMaxDifference_m);

  // mInitialScan is used to compare values with.
  lObjectDetection.mInitialScan.addLidarData(0.0, 1.00);
  lObjectDetection.mInitialScan.addLidarData(0.1, 4.01);
  lObjectDetection.mInitialScan.addLidarData(0.2, 4.02);
  lObjectDetection.mInitialScan.addLidarData(0.3, 4.03);
  lObjectDetection.mInitialScan.addLidarData(0.4, 1.04);

  std::pair<double, double> lNeighbours =
      lObjectDetection.getSurroundingDistances(0.05);

  /** Neighbours of 0.05 degrees should be 0.0 and 0.1 of mInitialScan, so
  expected corresponding distances are their 1.00 and 4.01 */
  // ASSERT_NEAR(1.00, lNeighbours.first,
  // std::numeric_limits<double>::epsilon());
  ASSERT_NEAR(4.01, lNeighbours.second, std::numeric_limits<double>::epsilon());
}

TEST(ObjectDetection, isAngleDifferent)
{
  const double cMaxDistanceDifference_m = 0.2;

  ObjectDetection lObjectDetection(
      cMaxDistanceDifference_m, true,
      objectdetection_unittest_constants::cMinNumberAdjacentAngles,
      objectdetection_unittest_constants::cNumberOfInitialScanRounds);

  // mInitialScan is used to compare values with.
  lObjectDetection.mInitialScan.addLidarData(0.0, 1.00);
  lObjectDetection.mInitialScan.addLidarData(0.1, 4.01);
  lObjectDetection.mInitialScan.addLidarData(0.2, 4.02);
  lObjectDetection.mInitialScan.addLidarData(0.3, 4.03);
  lObjectDetection.mInitialScan.addLidarData(0.4, 1.04);

  lObjectDetection.mInitialized = true;

  // Measurement (angle in radians => distance in meters)
  std::pair<double, double> lMeasurement =
      std::pair<double, double>(0.05, 1.00);

  // We expect false, as the value of angle 0.0 has a corresponding distance
  // of 1.00, which is equal to lMeasurement.second
  EXPECT_FALSE(lObjectDetection.isAngleDifferent(lMeasurement));

  lMeasurement = std::pair<double, double>(0.15, 2.00);

  // We expect true, as the value of distances corresponding to angles 0.1 and
  // 0.2 are different compared to lMeasurement.second
  EXPECT_TRUE(lObjectDetection.isAngleDifferent(lMeasurement));

  lMeasurement = std::pair<double, double>(0.9, 1.02);

  // We expect false, this measurement should be compared with distances at 0.4
  // (lower) and 0.0 (upper)
  EXPECT_FALSE(lObjectDetection.isAngleDifferent(lMeasurement));
}

TEST(ObjectDetection, convertVectorsTo2D)
{
  ObjectDetection lObjectDetection(
      objectdetection_unittest_constants::cMaxDifference_m, true,
      objectdetection_unittest_constants::cMinNumberAdjacentAngles,
      objectdetection_unittest_constants::cNumberOfInitialScanRounds);

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
  ObjectDetection lObjectDetection(
      objectdetection_unittest_constants::cMaxDifference_m, true,
      objectdetection_unittest_constants::cMinNumberAdjacentAngles,
      objectdetection_unittest_constants::cNumberOfInitialScanRounds);

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