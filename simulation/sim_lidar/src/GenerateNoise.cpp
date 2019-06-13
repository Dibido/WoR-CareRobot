#include "sim_lidar/GenerateNoise.hpp"

generate::GenerateNoise::GenerateNoise(double aMean, double aDevation)
    : mMean(aMean), mDeviation(aDevation)

{
}

void generate::GenerateNoise::generateNoiseSample(double aMean,
                                                  double aDeviation)
{
  for (int i = 0; i < lidar::cMeasurements; ++i)
  {

    std::mt19937 lGen;
    lGen.seed(i);

    std::normal_distribution<double> ld(aMean, aDeviation);

    mStep = ld(lGen);
    mNoise.push_back(mStep);
  }
}
