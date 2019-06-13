#ifndef LIDARCONST_HPP
#define LIDARCONST_HPP

#include <string>
#include <vector>

namespace lidar
{
  const int cMeasurements = 360;
  const std::string cDataset1 =
      "src/wor-18-19-s2/simulation/sim_lidar/datasets/lidardataset1.txt";
  const std::string cDataset2 =
      "src/wor-18-19-s2/simulation/sim_lidar/datasets/lidardataset2.txt";
  const double cMean = 0.980999;
  const double cDeviation = 0.00274328;

} // namespace lidar
#endif