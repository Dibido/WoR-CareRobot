#include "sim_lidar/CalculateData.hpp"

calculate::Calculatedata::Calculatedata(std::vector<double> aTheta,
                                        std::vector<double> aDistance)
    : mTheta_(aTheta), mDistance_(aDistance)
{
}

void calculate::Calculatedata::calculateAverage()
{
  double lSum = 0, counter = 0;
  for (const auto& t : mTheta_)
  {
    lSum += t;
  }
  mSumTheta_ = lSum / counter;
}

double calculate::Calculatedata::calculateAvagerageDistance()
{
    mSumTheta_
}