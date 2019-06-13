
#include "sim_lidar/CalculateData.hpp"
#include "sim_lidar/GenerateNoise.hpp"

int main(int argc, char** argv)
{

  calculate::Calculatedata lCalc;
  lCalc.fillVector(
      "wor-18-19-s2/simulation/sim_lidar/datasets/"
      "lidardataset1.txt");
  // lCalc.processData();
  lCalc.calculateStepSize();
  lCalc.calculateAverage();

  // generate::GenerateNoise lNoise;

  // lNoise.GenerateNoiseSample(lCalc.mMean, lCalc.mDeviation);

  return 0;
}
