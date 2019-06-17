#include "lidar_application/ObjectDetection.hpp"

namespace lidar_application
{
  ObjectDetection::ObjectDetection()
      : mInitialized(false),
        mMaxDistanceDifference_m(
            objectdetection_constants::cDefaultMaxDistanceDifference_m),
        mMaxReliableDistance_m(
            objectdetection_constants::cMaxReliableDistance_m),
        mIgnoreSmallObjects(false),
        mObjectMinNumberOfAdjacentMeasurements(
            objectdetection_constants::cObjectMinNumberOfAdjacentMeasurements),
        mAmountOfInitialScansRequired(
            objectdetection_constants::cDefaultAmountOfInitialScans)
  {
  }

  ObjectDetection::ObjectDetection(const double& aMaxDistanceDifference_m)
      : mInitialized(false),
        mMaxDistanceDifference_m(aMaxDistanceDifference_m),
        mMaxReliableDistance_m(
            objectdetection_constants::cMaxReliableDistance_m),
        mIgnoreSmallObjects(false),
        mObjectMinNumberOfAdjacentMeasurements(
            objectdetection_constants::cObjectMinNumberOfAdjacentMeasurements),
        mAmountOfInitialScansRequired(
            objectdetection_constants::cDefaultAmountOfInitialScans)
  {
  }

  ObjectDetection::ObjectDetection(
      const double& aMaxDistanceDifference_m,
      bool aIgnoreSmallObjects,
      const unsigned int& aObjectMinNumberOfAdjacentAngles,
      const unsigned int& aAmountOfInitialScansRequired)
      : mInitialized(false),
        mMaxDistanceDifference_m(aMaxDistanceDifference_m),
        mMaxReliableDistance_m(
            objectdetection_constants::cMaxReliableDistance_m),
        mIgnoreSmallObjects(aIgnoreSmallObjects),
        mObjectMinNumberOfAdjacentMeasurements(
            aObjectMinNumberOfAdjacentAngles),
        mAmountOfInitialScansRequired(aAmountOfInitialScansRequired)
  {
  }

  void ObjectDetection::run()
  {
    // Hertz rate of 100, max 10 ms in between cycles
    ros::Rate lRate(100);

    unsigned int lInitialScanIterations = 0;

    while (ros::ok())
    {
      ros::spinOnce();

      if (mDataHandler.isNewDataAvailable())
      {
        if (mInitialized)
        {
          mMostRecentScan = mDataHandler.getLidarData();

          detectObjects();

          mDataHandler.publishData(mDetectedObjects,
                                   objectdetection_constants::cLidarHeight_m);
        }
        else
        {
          mInitialScan.addLidarData(mDataHandler.getLidarData().mMeasurements);

          lInitialScanIterations++;

          if (lInitialScanIterations >= mAmountOfInitialScansRequired)
          {
            mInitialized = true;
          }
        }
      }
      lRate.sleep();
    }
  }

  void ObjectDetection::detectObjects()
  {
    // Checking preconditions
    if ((mInitialScan.mMeasurements.size() == 0) ||
        (mMostRecentScan.mMeasurements.size() == 0))
    {
      throw std::logic_error("Preconditions of detectObjects weren't met");
    }

    // Will contain a list of (front-)centerpoints of objects [Angle(key) ->
    // Distance(value)]
    std::vector<std::pair<double, double>> lObjectList;

    // Boolean shows if previous distances compared where within (false) the max
    // distance range or not (true).
    bool lLastComparisonDifferent = false;

    // Will contain all measurements that belong to an object
    LidarData lObject;

    /* The first object that is detected, this is stored seperately
    so if we detect an object at end of range, we can check if this is the same
    object as this firstobject. NOTE: In this context, a object is only to be
    considered a 'first object' if it is detected from the very first
    measurement onwards. */
    LidarData lBeginRangeObject;

    double lPreviousDistance_m = 0.0;

    for (std::map<double, double>::iterator lIt =
             mMostRecentScan.mMeasurements.begin();
         lIt != mMostRecentScan.mMeasurements.end(); ++lIt)
    {
      // double lInitialDistance_m = mInitialScan.mDistances_m.at(i);

      double lCurrentDistance_m = lIt->second;
      double lCurrentAngle_m = lIt->first;

      if (isAngleDifferent((*lIt)))
      {
        // Current measurement wasn't taken of the same object as previous angle
        if (std::abs(lCurrentDistance_m - lPreviousDistance_m) >
            mMaxDistanceDifference_m)
        {
          // If lObject contains valid info (it won't at first iteration)
          if ((lObject.mMeasurements.size() > 0))
          {
            /** If begin range object hasn't been stored yet and lObject
            is detected from the very first measurement angle */
            if (lBeginRangeObject.mMeasurements.size() == 0 &&
                lObject.mMeasurements.begin()->first ==
                    mMostRecentScan.mMeasurements.begin()->first)
            {
              // If we want to ignore small objects, and objectsize is lower
              // then required
              if (mIgnoreSmallObjects && isSmallObject(lObject))
              {
                lObject.reset();
              }
              else
              {
                lBeginRangeObject = lObject;
                lObject.reset();
              }
            }
            else // Not the first object
            {
              // If we want to ignore small objects, and objectsize is lower
              // then required
              if (mIgnoreSmallObjects && isSmallObject(lObject))
              {
                lObject.reset();
              }
              else
              {
                // Add centerpoint of this object to list
                lObjectList.push_back(getAverageMeasurement(lObject));
                lObject.reset();
              }
            }

            lObject.reset();
          }
        }

        lObject.addLidarData(lCurrentAngle_m, lCurrentDistance_m);
        lLastComparisonDifferent = true;
      }
      else
      {
        if (lLastComparisonDifferent == true)
        {
          /** If begin range object hasn't been stored yet and lObject
          is detected from the very first measurement angle */
          if (lBeginRangeObject.mMeasurements.size() == 0 &&
              lObject.mMeasurements.begin()->first ==
                  mMostRecentScan.mMeasurements.begin()->first)
          {
            // If we want to ignore small objects, and objectsize is lower then
            // required
            if (mIgnoreSmallObjects && isSmallObject(lObject))
            {
              lObject.reset();
            }
            else
            {
              lBeginRangeObject = lObject;
            }
          }
          else // Not begin range object
          {
            // If we want to ignore small objects, and objectsize is lower then
            // required
            if (mIgnoreSmallObjects && isSmallObject(lObject))
            {
              lObject.reset();
            }
            else
            {
              // Add centerpoint of this object to list
              lObjectList.push_back(getAverageMeasurement(lObject));
            }
          }

          lObject.reset();

          lLastComparisonDifferent = false;
        }
      }

      lPreviousDistance_m = lCurrentDistance_m;
    }

    // There has been detected a object in begin of range
    if (lBeginRangeObject.mMeasurements.size() > 0)
    {
      // And there has also been detected a object at the end of the range
      if (lObject.mMeasurements.size() > 0)
      {
        // If the last measurement of this object, is close to the first
        // measurement of the object detected in the beginning of the range
        std::pair<double, double> lObjectsLastMeasurement =
            *(lObject.mMeasurements.end()--);

        if (std::abs(lObjectsLastMeasurement.second -
                     lBeginRangeObject.mMeasurements.begin()->second) <=
            mMaxDistanceDifference_m)
        {
          // Add the begin range object data to this object, as it must be
          // measurements of the same object
          lObject.addLidarData(lBeginRangeObject.mMeasurements);
        }
        else
        {
          // Store the object detected in begin of range seperately
          lObjectList.push_back(getAverageMeasurement(lBeginRangeObject));
        }

        // If we want to ignore small objects, and objectsize is lower then
        // required
        if (mIgnoreSmallObjects && isSmallObject(lObject))
        {
          lObject.reset();
        }
        else
        {
          // Add centerpoint of this object to list
          lObjectList.push_back(getAverageMeasurement(lObject));
        }
      }
      // There was no object detected at the end of the range, the object
      // detected at begin of range must be isolated and can be added on its
      // own.
      else
      {
        lObjectList.push_back(getAverageMeasurement(lBeginRangeObject));
      }
    }

    mDetectedObjects = convertVectorsTo2D(filterFarObjects(lObjectList));
  }

  std::vector<std::pair<double, double>> ObjectDetection::convertVectorsTo2D(
      std::vector<std::pair<double, double>> aData) const
  {
    std::vector<std::pair<double, double>> lReturnObjects;

    for (size_t i = 0; i < aData.size(); ++i)
    {
      double lDistance_m = aData.at(i).second;

      /* Convert angle in such a way that we can use sin/cos functions
      correctly, compensating for the fact the lidars view starts north and then
      goes in a clockwise circle, while the unit circle starts east and then
      moves anti-clockwise */
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

    int lSampleSize = static_cast<int>(aData.mMeasurements.size());

    for (std::map<double, double>::iterator lIt = aData.mMeasurements.begin();
         lIt != aData.mMeasurements.end(); ++lIt)
    {
      lSumAngles += lIt->first;
      lSumDistance_m += lIt->second;
    }

    if (lSampleSize > 0)
    {
      lAverageAngle = lSumAngles / lSampleSize;
      lAverageDistance_m = lSumDistance_m / lSampleSize;
    }

    return std::make_pair(lAverageAngle, lAverageDistance_m);
  }

  bool ObjectDetection::isAngleDifferent(
      const std::pair<double, double>& aMeasurement) const
  {
    const double lAngle = aMeasurement.first;
    const double lDistance_m = aMeasurement.second;

    const std::pair<double, double> lSurroundingDistances =
        getSurroundingDistances(lAngle);

    const double lDifferenceToLowerNeighbour_m =
        std::abs(lDistance_m - lSurroundingDistances.first);
    const double lDifferenceToUpperNeighbour_m =
        std::abs(lDistance_m - lSurroundingDistances.second);

    // Given measurement is too far out of line with data from mInitialScan
    if ((lDifferenceToLowerNeighbour_m > mMaxDistanceDifference_m) &&
        (lDifferenceToUpperNeighbour_m > mMaxDistanceDifference_m))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  std::pair<double, double>
      ObjectDetection::getSurroundingDistances(const double aAngle) const
  {
    if (!(static_cast<int>(mInitialScan.mMeasurements.size()) >= 2))
    {
      throw std::logic_error(
          "Preconditions of getSurroundingDistances not met");
    }

    if ((aAngle < 0.0) || (aAngle > (2 * M_PI)))
    {
      throw std::range_error("Angle isn't a value in range [0.0 -> 2*PI]");
    }

    double lLowerNeighbourDistance_m = 0.0;
    double lUpperNeighbourDistance_m = 0.0;

    auto lIterator = mInitialScan.mMeasurements.lower_bound(aAngle);

    // If there doesn't exist a key with a equal or higher value then aAngle:
    if ((lIterator->first == 0.0) && lIterator->second == 0.0)
    {
      // It will probably be a value close to the maximum of 2 * PI

      // Take the highest angle as lower neighbour
      lLowerNeighbourDistance_m = (--mInitialScan.mMeasurements.end())->second;

      // Take the lowest angle as upper neighbour
      lUpperNeighbourDistance_m = mInitialScan.mMeasurements.begin()->second;
    }
    else // There has been found a key with a equal or higher value then aAngle:
    {
      lUpperNeighbourDistance_m = lIterator->second;

      // If there exists a lower angle, use that.
      if (!(lIterator == mInitialScan.mMeasurements.begin()))
      {
        auto lPreviousElementIterator = lIterator;
        lPreviousElementIterator--;

        lLowerNeighbourDistance_m = lPreviousElementIterator->second;
      }
      // Otherwise take the highest angle as lower neighbour.
      else
      {
        lLowerNeighbourDistance_m =
            (mInitialScan.mMeasurements.end()--)->second;
      }
    }

    return std::pair<double, double>(lLowerNeighbourDistance_m,
                                     lUpperNeighbourDistance_m);
  }

  std::vector<std::pair<double, double>> ObjectDetection::filterFarObjects(
      const std::vector<std::pair<double, double>>& aObjectList) const
  {
    std::vector<std::pair<double, double>> lReturnObjects;

    for (size_t lIndex = 0; lIndex < aObjectList.size(); ++lIndex)
    {
      std::pair<double, double> lPair = aObjectList.at(lIndex);

      if (lPair.second <= mMaxReliableDistance_m)
      {
        lReturnObjects.push_back(lPair);
      }
    }

    return lReturnObjects;
  }

  void ObjectDetection::printPublishData() const
  {
    ROS_INFO("Detected objects:");

    for (const auto& lPair : mDetectedObjects)
    {
      ROS_INFO("(%f, %f)", lPair.first, lPair.second);
    }
    ROS_INFO("----------------------");
  }

  bool ObjectDetection::isSmallObject(const LidarData& lObject) const
  {
    if (lObject.mMeasurements.size() < mObjectMinNumberOfAdjacentMeasurements)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
} // namespace lidar_application
