#ifndef OBJECTDETECTION_H_
#define OBJECTDETECTION_H_

#include "lidar_application/DataHandler.hpp"
#include "lidar_application/LidarData.hpp"
#include <cmath>
#include <map>
#include <math.h>
#include <vector>

namespace lidar_application
{
  namespace objectdetection_constants
  {
    // Maximum difference allowed between measurement for them to be considered
    // as measurements of the same object
    const double cDefaultMaxDistanceDifference_m = 0.2;

    // Z position of lidar
    const double cLidarHeight_m = 0.5;
  } // namespace objectdetection_constants

  class ObjectDetection
  {
      public:
    /**
     * @brief Constructor
     * @param aMaxDistanceDifference_m - This variable describes the max amount
     * of difference in meters between two adjacent measurements to still assume
     * its the same object. Example: [Theta => Distance] [0.0 => 1.5],
     * [1.0, 1.6]. The difference in meters here is 0.1, if that is under or
     * equal to aMaxDistanceDifference_m, both these measurements are taken of
     * the same object.   */
    explicit ObjectDetection(
        double aMaxDistanceDifference_m =
            objectdetection_constants::cDefaultMaxDistanceDifference_m);

    ~ObjectDetection() = default;

    /**
     * @brief Run function, blocking function that handles all logic
     */
    void run();

      private:
    /**
     * @brief Detects objects. Uses mInitialScan for comparison.
     * @pre: mInitialScan contains valid information (measurents between 0 and
     * 2*PI radians), mMostRecentScan contains valid data.
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
     * @brief Checks whether a measurement is different based on comparison with
     * mInitialScan. method uses getSurroundingDistances() and compares the
     * distance of aMeasurement with those. If difference between aMeasurement's
     * distance and both surrounding distances is greater then
     * mMaxDistanceDifference_m, the measurement is considered 'different'.
     * Otherwise, the measurement could very well be in line with the measurents
     * of mInitialScan and the measurement won't be considered different.
     * @param aMeasurement - Given measurement (angle in radians, distance in
     * meters)
     * @return true if angle is different, false otherwise.
     */
    bool isAngleDifferent(const std::pair<double, double>& aMeasurement) const;

    /**
     * @brief Function returns the distance of the two angles from mInitialScan
     * that are the closest (surrounding) the given aAngle value. If
     * mInitialScan has measurements on angles 0.0 and 1.1, and aAngle is 0.5,
     * the distances that were measured at 0.0 and 1.1 are returned.
     * @param aAngle - The given angle in radians
     * @return std::pair<double, double>
     */
    std::pair<double, double>
        getSurroundingDistances(const double aAngle) const;

    /**
     * @brief Prints out mDetectedObjects
     */
    void printPublishData() const;

    bool mInitialized;

    // Contains the data of the first 360 degrees scan, set during
    // initialization.
    LidarData mInitialScan;

    // Contains the data of the most recent 360 degrees scan performed.
    LidarData mMostRecentScan;

    // Contains data of the detect objects. detectObjects() stores the data
    // (format: angle(rad), distance(meter));
    std::vector<std::pair<double, double>> mDetectedObjects;

    DataHandler mDataHandler;

    // Maximum difference allowed between adjacent/previous measurements for
    // them to be considered as measurements of the same object.
    const double mMaxDistanceDifference_m;
  };
} // namespace lidar_application

#endif // OBJECTDETECTION_H_