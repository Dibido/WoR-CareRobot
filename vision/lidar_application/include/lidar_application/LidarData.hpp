#ifndef LIDARDATA_H_
#define LIDARDATA_H_

#include <iostream>
#include <map>
#include <vector>

namespace lidar_application
{
  /**
   * @brief This struct contains data of a full 360-degree scan of a lidar.
   * mMeasurements contains measurements in form of Angle in radians (key) =>
   * distance in meters (value)
   */
  struct LidarData
  {
    // Measurements, in form of angle in radians (key) => distance in meters
    // (value)
    std::map<double, double> mMeasurements;

    /**
     * @brief mMeasurements is cleared
     * @pre: -
     * @post: mMeasurements is empty
     */
    void reset();

    /**
     * @brief Default constructor
     */
    LidarData();

    ~LidarData() = default;

    /**
     * @brief Construct a new Lidar Data object
     * @param aMeasurements - The measurements in format Angle (key) => Distance
     * in meters (value) all angles must be >= 0.0, and <= 2*M_PI radians.
     */
    LidarData(const std::map<double, double> aMeasurements);

    /**
     * @brief Adds samples to the existing angles/distances,
     * aAngles and aDistances_m MUST be of
     * same size, otherwise an exception is thrown.
     * @pre: -
     * @post: Given values are added to mMeasurements
     * @param aAngles - angles in radians, must be >= 0.0 and <= 2*PI
     * @param aDistances_m - corresponding distances in meters
     */
    void addLidarData(std::vector<double>& aAngles,
                      std::vector<double>& aDistances_m);

    /**
     * @brief Adds sample to the existing measurements
     * @pre: -
     * @post: Given values are added to mMeasurements
     * @param aAngles - angle in radians, must be >= 0.0 and <= 2*PI
     * @param aDistances_m - corresponding distances in meters
     */
    void addLidarData(double aAngle, double aDistance_m);
  };
} // namespace lidar_application

#endif // LIDARDATA_H_