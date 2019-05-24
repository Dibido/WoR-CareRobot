#ifndef OBJECTDETECTION_H_
#define OBJECTDETECTION_H_

#include "DataHandler.hpp"
#include "LidarData.hpp"

#include <cmath>
#include <iostream>
#include <math.h>
#include <vector>

namespace lidar_application
{
namespace objectdetection_constants
{
  // Maximum difference allowed between measurement for them to be considered as
  // measurements of the same object
  const double cDefaultMaxDistanceDifference_m = 0.2;

  // Z position of lidar
  const double cLidarHeight_m = 0.5;
} // namespace ObjectDetectionConstants

class ObjectDetection
{
    public:
  /**
   * @brief Constructor
   * @param aMaxDistanceDifference_m - This variable describes the max amount of difference in meters between
   * two adjacent measurements to still assume its the same object. Example: [Theta => Distance]
   * [0.0 => 1.5], [1.0, 1.6]. The difference in meters here is 0.1, if that is under or equal to aMaxDistanceDifference_m,
   * both these measurements are taken of the same object.   */
  explicit ObjectDetection(double aMaxDistanceDifference_m = objectdetection_constants::cDefaultMaxDistanceDifference_m);

  ~ObjectDetection() = default;

  /**
   * @brief Run function, blocking function that handles all logic
   */
  void run();

    private:
  /**
   * @brief Detects objects. Uses mInitialScan for comparison.
   * @pre: mInitialScan is made, mMostRecentScan data is accurate. Size
   * of all of their vectors must be greather then 0.
   * @post: mDetectedObjects data contains a list of
   * objects, with angle and distance to each centerpoint of an object.
   */
  virtual void detectObjects();

  /**
   * @brief Function converts data in vector format (theta, distance) to
   * objects position (x, y) relative to the lidar.
   * @pre: -
   * @post: -
   * @param aData - Valid data format (theta, distance). Angles must be in
   * radians.
   * @return A list of positions of objects (X,Y).
   */
  std::vector<std::pair<double, double>>
      convertVectorsTo2D(std::vector<std::pair<double, double>> aData) const;

  /**
   * @brief Function returns the average angle (key) and distance (value) of a
   * data set.
   * @pre: -
   * @post: -
   * @aData - LidarData, must have equal amount of angles and distances.
   * @return - Average <angle, distance> of the given set.
   */
  std::pair<double, double> getAverageMeasurement(LidarData& aData) const;

  /**
   * @brief Prints out mPublishData
   */
  void printPublishData() const; 

  bool mInitialized;

  // Contains the data of the first 360 degrees scan, set during initialization.
  LidarData mInitialScan;

  // Contains the data of the most recent 360 degrees scan performed.
  LidarData mMostRecentScan;

  std::vector<std::pair<double, double>> mDetectedObjects;

  // Contains the data that will be published
  std::vector<std::pair<double, double>> mPublishData;

  DataHandler mDataHandler;

  // Maximum difference allowed between different measurements for them to be considered as
  // measurements of the same object. If (0.0 => 1.50 and 1.0 =>)
  const double mMaxDistanceDifference_m;
};
}

#endif // OBJECTDETECTION_H_