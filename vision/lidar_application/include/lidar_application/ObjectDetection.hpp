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

    // Reality has shown that the lidar can measure up to around 5 meters
    // reliably. See
    // http://wor.wiki.icaprojecten.nl/confluence/display/EBGURG/Vision+-+Onderzoek+toepassingen+lidar
    const double cMaxReliableDistance_m = 5.0;

    /** Is used to determine whether a object should be filtered out because of
    its size. If an object is detected, but it has only 1 different angle
    compared to mInitialScan, it will be left out since 1 is <
    cObjectMinNumberOfAdjacentMeasurements */
    const unsigned int cObjectMinNumberOfAdjacentMeasurements = 2;
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
     * the same object.
     * @param aIgnoreSmallObjects - When this is true, objects with less then 2
     * adjacent measurements are ignored. This should
     * filter out thin real-life objects like a cable */
    ObjectDetection(
        double aMaxDistanceDifference_m =
            objectdetection_constants::cDefaultMaxDistanceDifference_m,
        bool aIgnoreSmallObjects = false);

    ~ObjectDetection() = default;

    /**
     * @brief Run function, blocking function that handles all logic
     */
    void run();

    // Debug variable for minimum amount of adjacent measurements
    unsigned int cObjectMinNumberOfAdjacentMeasurementsDebug = 2;

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
     * @pre mInitialScan must be filled with atleast 2 samples
     * @post -
     * @param aAngle - The given angle in radians, must be >= 0.0 and <= 2 * PI
     * @return std::pair<double, double>
     */
    std::pair<double, double>
        getSurroundingDistances(const double aAngle) const;

    /**
     * @brief Filters out any objects with a distance greater then
     * mMaxReliableDistance_m
     * @param aObjectList - Objectlist in form (theta in radians, distance in
     * meters)
     * @return List of objects in same form as aObjectList, only without the
     * objects that had a distance greater then mMaxReliableDistance_m
     */
    std::vector<std::pair<double, double>> filterFarObjects(
        const std::vector<std::pair<double, double>>& aObjectList) const;

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

    // Maximum distance that the lidar can reliable measure.
    const double mMaxReliableDistance_m;

    /** If this boolean is true, objects require atleast 2 adjacent
    measurements. this means that if just a single angle is considered
    different, this won't be registered as an object. This filters out really
    small objects like a cable */
    bool mIgnoreSmallObjects;
  };
} // namespace lidar_application

#endif // OBJECTDETECTION_H_
