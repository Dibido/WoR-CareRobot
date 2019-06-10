
#include "sim_lidar/CalculateData.hpp"
#include "sim_lidar/GenerateNoise.hpp"

int main(int argc, char** argv)
{

  calculate::Calculatedata lCalc;
  lCalc.processData();

  generate::GenerateNoise lNoise;

   lNoise.GenerateNoiseSample(lCalc.mMean, lCalc.mDeviation);

  return 0;
}
