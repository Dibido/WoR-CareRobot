#include "sim_lidar/GenerateNoise.hpp"

generate::GenerateNoise::GenerateNoise(double aMean, double aDevation)
    : mMean(aMean), mDeviation(aDevation)

{
  std::cout << "mean:" << mMean << "devation:" << mDeviation << std::endl;
}

void generate::GenerateNoise::GenerateNoiseSample(double aMean,
                                                  double aDeviation)
{
  for (int i = 0; i < lidar::cMeasurements; ++i)
  {

    std::mt19937 gen(mrd());

    std::normal_distribution<double> d(aMean, aDeviation);

    mStep = d(gen);
    mNoise.push_back(mStep);
  }
  for (unsigned int i = 0; i < mNoise.size(); ++i)
  {
    std::cout << mNoise[i] << std::endl;
  }
}
