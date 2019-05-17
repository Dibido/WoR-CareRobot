#include "../include/ObjectDetection.h"

ObjectDetection::ObjectDetection() : mInitialized(false)
{
}

ObjectDetection::~ObjectDetection()
{
}

void ObjectDetection::run()
{
  while (true)
  {
    bool lNewData = false;
    // TO-DO  check for new data

    if (lNewData)
    {
      if (mInitialized)
      {
        // Processdata

        // Publish objects
      }
      else
      {
        // mInitialScan =
      }
    }
  }
}

void ObjectDetection::detectObjects()
{
  // Checking preconditions
  if ((mInitialScan.mDistances_m.size() == 0) ||
      (mMostRecentScan.mDistances_m.size() == 0) ||
      (mInitialScan.mDistances_m.size() != mMostRecentScan.mDistances_m.size()) ||
      (!validateLidarData(mInitialScan)) ||
      (!validateLidarData(mMostRecentScan)))
  {
    throw std::logic_error("Preconditions of detectObjects weren't met");
  }

  // Will contain a list of centerpoints of objects [Angle(key) -> Distance(value)]
  std::vector<std::pair<double, double>> lObjectList;

  // Boolean shows if previous distances compared where within (false) the max distance range or not (true). 
  bool lLastComparisonDifferent = false;

  // Will contain all measurements that belong to an object
  LidarData lObject;

  double lPreviousDistance_m = 0.0;

  for (int i = 0; i < mMostRecentScan.mDistances_m.size(); ++i)
  {
    double lInitialDistance_m = mInitialScan.mDistances_m.at(i);
    double lInitialAngle = mInitialScan.mAngles.at(i);

    double lCurrentDistance_m = mMostRecentScan.mDistances_m.at(i);
    double lCurrentAngle_m = mMostRecentScan.mAngles.at(i);
    
    double lDistanceDifference_m = std::abs(lInitialDistance_m - lCurrentDistance_m);

    // There is a change compared to initial scan
    if((lDistanceDifference_m > ObjectDetectionConstants::cMaxDistanceDifference_m))
    {
      // Current measurement can't be taken of the same object as previous iteration
      if(std::abs(lCurrentDistance_m - lPreviousDistance_m) > ObjectDetectionConstants::cMaxDistanceDifference_m)
      {
        // If lObject contains valid info (it won't at first iteration)
        if((lObject.mDistances_m.size() > 0))
        {
          // Add centerpoint of this object to list
          lObjectList.push_back(getAverageMeasurement(lObject));
          lObject.reset();
        }
      }

      lObject.mAngles.push_back(lCurrentAngle_m);  

      lLastComparisonDifferent = true;
    }
    else 
    {
      if(lLastComparisonDifferent == true)
      {
        // Add centerpoint of this object to the list
        lObjectList.push_back(getAverageMeasurement(lObject));
        lObject.reset();

        lLastComparisonDifferent = false;
      }
    }

    lPreviousDistance_m = lCurrentDistance_m;
  }
}

std::pair<double, double> ObjectDetection::getAverageMeasurement(LidarData& aData) const
{
  if(!validateLidarData(aData))
  {
    throw std::logic_error("getAverageMeasurement was used wrongly, aData.mAngles size is not equal to aData.mDistances_m");
  }

  double lAverageAngle = 0.0;
  double lAverageDistance_m = 0.0;

  double lSumAngles = 0.0;
  double lSumDistance_m = 0.0;

  int lSampleSize = aData.mAngles.size();

  for(int i = 0; i < aData.mDistances_m.size(); ++i)
  {
    lSumAngles += aData.mAngles.at(i);
    lSumDistance_m += aData.mDistances_m.at(i);
  }

  if(lSampleSize > 0)
  {
    lAverageAngle = lSumAngles / lSampleSize;
    lAverageDistance_m = lSumDistance_m / lSampleSize;
  }

  return std::make_pair(lAverageAngle, lAverageDistance_m);
}

bool ObjectDetection::validateLidarData(LidarData& aData) const
{
  if(aData.mAngles.size() == aData.mDistances_m.size())
  {
    return true;
  }
  else 
  {
    return false;
  }
}