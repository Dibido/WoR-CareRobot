#include "../include/ObjectDetection.hpp"

// Declaration of namespace variables
const double ObjectDetectionConstants::cDefaultMaxDistanceDifference_m = 0.2;
const double ObjectDetectionConstants::cLidarHeight_m = 0.5;

ObjectDetection::ObjectDetection(double aMaxDistanceDifference_m) : mInitialized(false), mMaxDistanceDifference_m(aMaxDistanceDifference_m)
{
}

ObjectDetection::~ObjectDetection()
{
}

void ObjectDetection::run()
{
  while (true)
  {
    // Sleep for a millisec
    ros::Duration(0.001).sleep();

    ros::spinOnce();

    if (mDataHandler.isNewDataAvailable())
    {
      if (mInitialized)
      {
        mMostRecentScan = mDataHandler.getLidarData();

        detectObjects();

        printPublishData();
        mDataHandler.publishData(mDetectedObjects, ObjectDetectionConstants::cLidarHeight_m);
      }
      else
      {
        mInitialScan = mDataHandler.getLidarData();
        mInitialized = true;
      }
    }
  }
}

void ObjectDetection::detectObjects()
{
  // Checking preconditions
  if ((mInitialScan.mDistances_m.size() == 0) ||
      (mMostRecentScan.mDistances_m.size() == 0))
  {
    throw std::logic_error("Preconditions of detectObjects weren't met");
  }

  // Will contain a list of centerpoints of objects [Angle(key) ->
  // Distance(value)]
  std::vector<std::pair<double, double>> lObjectList;

  // Boolean shows if previous distances compared where within (false) the max
  // distance range or not (true).
  bool lLastComparisonDifferent = false;

  // Will contain all measurements that belong to an object
  LidarData lObject;

  double lPreviousDistance_m = 0.0;

  for (size_t i = 0; i < mMostRecentScan.mDistances_m.size(); ++i)
  {
    double lInitialDistance_m = mInitialScan.mDistances_m.at(i);

    double lCurrentDistance_m = mMostRecentScan.mDistances_m.at(i);
    double lCurrentAngle_m = mMostRecentScan.mAngles.at(i);

    double lDistanceDifference_m = lInitialDistance_m - lCurrentDistance_m;

    // There is a positive change compared to initial scan (object came closer)
    if ((lDistanceDifference_m >
         mMaxDistanceDifference_m))
    {
      // Current measurement wasn't taken of the same object as previous angle
      if (std::abs(lCurrentDistance_m - lPreviousDistance_m) >
          mMaxDistanceDifference_m)
      {
        // If lObject contains valid info (it won't at first iteration)
        if ((lObject.mDistances_m.size() > 0))  
        {
          // Add centerpoint of this object to list
          lObjectList.push_back(getAverageMeasurement(lObject));
          lObject.reset();
        }
      }

      lObject.mAngles.push_back(lCurrentAngle_m);
      lObject.mDistances_m.push_back(lCurrentDistance_m);

      lLastComparisonDifferent = true;
    }
    else
    {
      if (lLastComparisonDifferent == true)
      {
        // Add centerpoint of this object to the list
        lObjectList.push_back(getAverageMeasurement(lObject));
        lObject.reset();

        lLastComparisonDifferent = false;
      }
    }

    lPreviousDistance_m = lCurrentDistance_m;
  }

  mPublishData = convertVectorsTo2D(lObjectList);
}

std::vector<std::pair<double, double>> ObjectDetection::convertVectorsTo2D(
    std::vector<std::pair<double, double>> aData) const
{
  std::vector<std::pair<double, double>> lReturnObjects;

  for (size_t i = 0; i < aData.size(); ++i)
  {
    double lDistance_m = aData.at(i).second;

    /* Convert angle in such a way that we can use sin/cos functions correctly,
    compensating for the fact the lidars view starts north and then goes in a
    clockwise circle, while the unit circle starts east and then moves
    anti-clockwise */
    double lConvertedAngle = (aData.at(i).first - M_PI_2) * -1;

    double lObjectX = lDistance_m * cos(lConvertedAngle);
    double lObjectY = lDistance_m * sin(lConvertedAngle);

    lReturnObjects.push_back(std::make_pair(lObjectX, lObjectY));
  }

  return lReturnObjects;
}

std::pair<double, double>
    ObjectDetection::getAverageMeasurement(LidarData& aData) const
{
  double lAverageAngle = 0.0;
  double lAverageDistance_m = 0.0;

  double lSumAngles = 0.0;
  double lSumDistance_m = 0.0;

  int lSampleSize = static_cast<int>(aData.mAngles.size());

  for (size_t i = 0; i < aData.mDistances_m.size(); ++i)
  {
    lSumAngles += aData.mAngles.at(i);
    lSumDistance_m += aData.mDistances_m.at(i);
  }

  if (lSampleSize > 0)
  {
    lAverageAngle = lSumAngles / lSampleSize;
    lAverageDistance_m = lSumDistance_m / lSampleSize;
  }

  return std::make_pair(lAverageAngle, lAverageDistance_m);
}

void ObjectDetection::printPublishData() const
{
  std::cout << "Detected objects:" << std::endl;

  for(auto lPair : mPublishData)
  {
    std::cout << "(" << std::to_string(lPair.first) << "," << std::to_string(lPair.second) << ")" << std::endl;
  }
  
  std::cout << std::endl;
}