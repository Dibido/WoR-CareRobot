#ifndef OBJECTDETECTION_H_
#define OBJECTDETECTION_H_

#include "../include/LidarData.h"

#include <iostream>
#include <vector>
#include <cmath>

namespace ObjectDetectionConstants
{
  // Maximum difference allowed between measurement for them to be considered as measurements of the same object
  double cMaxDistanceDifference_m = 0.1;
}


class ObjectDetection
{
  public:
      
    /**
     * @brief Default constructor
     */
    ObjectDetection();
      
    ~ObjectDetection();
      
    /**
     * @brief Run function, blocking function that handles all logic
     */
    void run();

  private:
    
    /**
     * @brief Detects objects. Uses mInitialScan for comparison.
     * @precondition: mInitialScan is made, mMostRecentScan data is accurate. Size of all of their vectors
     * must be greather then 0, and size of all vectors must be equal.
     * @postcondition: mDetectedObjects data contains a list of 
     * objects, with angle and distance to each centerpoint of an object.
     */
    void detectObjects();

    /**
     * @brief Function returns the average angle (key) and distance (value) of a data set.
     * @precondition: -
     * @postcondition: -
     * @aData - LidarData, must have equal amount of angles and distances.
     * @return - Average <angle, distance> of the given set.
     */
    std::pair<double, double> getAverageMeasurement(LidarData& aData) const;

    /**
     * @brief Checks whether given LidarData has equally sized angle/distance vectors.
     * @param aData - LidarData
     * @return True if equal size, false otherwise
     */
    bool validateLidarData(LidarData& aData) const;

    std::vector<std::pair<double, double>> mDetectedObjects;

    bool mInitialized;
    
    // Contains the data of the first 360 degrees scan, set during initialization.
    LidarData mInitialScan;

    // Contains the data of the most recent 360 degrees scan performed.
    LidarData mMostRecentScan;
};

#endif // OBJECTDETECTION_H_