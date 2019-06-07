
#include "sim_lidar/CalculateData.hpp"

int main(int argc, char** argv)
{
  std::vector<double> measurements;
  calculate::Calculatedata lCalc;
  lCalc.processData(
      "src/wor-18-19-s2/simulation/sim_lidar/src/lidarData.txt");

  return 0;
}

