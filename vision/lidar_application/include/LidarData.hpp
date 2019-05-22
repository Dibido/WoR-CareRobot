#ifndef LIDARDATA_H_
#define LIDARDATA_H_

#include <vector>
#include <iostream>

/**
 * @brief This struct contains data of a full 360-degree scan of a lidar.
 * Values in mAngles and mDistances are coupled by index.
 * mAngles[0] = 0.1 and mDistances[0] = 1.25 means that on angle 0.1 degrees,
 * the measured distance was 1.25 meters.
 */
struct LidarData
{
  std::vector<double> mAngles;
  std::vector<double> mDistances_m;

  /**
   * @brief Clears mAngles and mDistances_m
   * @precondition: -
   * @postcondition: mAngles and mDistances_m contain no elements
   */
  void reset();

  /**
   * @brief Default constructor
   */
  LidarData();

  /**
   * @brief Construct a new Lidar Data object, aAngles and aDistances_m MUST be of 
   * same size, otherwise an exception is thrown.
   * @param aAngles - angles in radians
   * @param aDistances_m - corresponding distances in meters
   */
  LidarData(std::vector<double>& aAngles, std::vector<double>& aDistances_m);

  /**
   * @brief Adds samples to the existing angles/distances, 
   * aAngles and aDistances_m MUST be of 
   * same size, otherwise an exception is thrown.
   * @precondition: -
   * @postcondition: Given values are added to mAngles/mDistances_m
   * @param aAngles - angles in radians
   * @param aDistances_m - corresponding distances in meters
   */
  void addLidarData(std::vector<double>& aAngles, std::vector<double>& aDistances_m);

  /**
   * @brief Adds sample to the existing angles/distances
   * @precondition: -
   * @postcondition: Given values are added to mAngles/mDistances_m
   * @param aAngles - angle in radians
   * @param aDistances_m - corresponding distances in meters
   */
  void addLidarData(double aAngle, double aDistance_m);
};

#endif // LIDARDATA_H_