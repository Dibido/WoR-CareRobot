#include "sim_lidar/GenerateNoise.hpp"

generate::GenerateNoise::GenerateNoise(double aMean, double aDevation)
    : mMean(aMean), mDeviation(aDevation)

{
}

void generate::GenerateNoise::GenerateNoiseSample(double aMean,
                                                  double aDeviation)
{
  for (int i = 0; i < lidar::cMeasurements; ++i)
  {

    std::mt19937 gen;
    gen.seed(i);

    std::normal_distribution<double> d(aMean, aDeviation);

    mStep = d(gen);
    mNoise.push_back(mStep);
   
  }

}
