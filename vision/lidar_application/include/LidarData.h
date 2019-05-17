#ifndef LIDARDATA_H_
#define LIDARDATA_H_

#include <vector>

/**
 * @brief This struct contains data of a full 360-degree scan of a lidar.
 * Values in mAngles and mDistances are coupled by index.
 * mAngles[0] = 0.1 and mDistances[0] = 1.25 means that on angle 0.1 degrees,
 * the measured distance was 1.25 meters.
 */
struct LidarData
{
  std::vector<double> mAngles;
  std::vector<double> mDistances;
};

#endif // LIDARDATA_H_