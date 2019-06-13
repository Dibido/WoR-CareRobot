
#include "sim_lidar/CalculateData.hpp"
#include "sim_lidar/GenerateNoise.hpp"

int main(int argc, char** argv)
{

  calculate::Calculatedata lCalc;
  lCalc.fillVector(
      "wor-18-19-s2/simulation/sim_lidar/datasets/"
      "lidardataset2.txt");

  lCalc.radiansToDegrees();
  lCalc.calculateStepSize();
  lCalc.calculateAverage();
  lCalc.calculateDeviation();

  return 0;
}
