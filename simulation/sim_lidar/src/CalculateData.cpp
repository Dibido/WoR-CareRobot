#include "sim_lidar/CalculateData.hpp"
#include "sim_lidar/LidarConst.hpp"

#include <algorithm>
#include <math.h>

calculate::Calculatedata::Calculatedata()
    : mMeasurements(0.0),
      mStepSize(0, 0),
      mMean(0.0),
      mDeviation(0),
      mDefectiveMeasurement(0)
{
}

calculate::Calculatedata::Calculatedata(std::vector<double> aMeasurements)
    : mMeasurements(aMeasurements), mStepSize(0, 0), mMean(0.0), mDeviation(0)
{
}

void calculate::Calculatedata::fillVector(std::string aFile)
{

  std::ifstream lFile(aFile, std::ios::in);

  if (!lFile.is_open())
  {
    std::cerr << "There was a problem opening the input file!\n";
    exit(1);
  }

  double lNum = 0.0;

  while (lFile >> lNum)
  {
    mRadians.push_back(lNum);
  }
}

void calculate::Calculatedata::radiansToDegrees()
{

  auto lIterator = 0;
  for (const auto& m : mRadians)
  {
    // std::cout << m << std::endl;
    mMeasurements.push_back((m * 180) / M_PI);
    // std::cout << mMeasurements[lIterator] << std::endl;
    ++lIterator;
  }
}

void calculate::Calculatedata::calculateStepSize(unsigned int aLowerBound,
                                                 unsigned int aUpperBound)
{

  for (unsigned int i = 1; i < mMeasurements.size(); ++i)
  {
    if ((mMeasurements[i] - mMeasurements[i - 1]) > aLowerBound &&
        (mMeasurements[i] - mMeasurements[i - 1]) < aUpperBound)
    {
      mStepSize.push_back(mMeasurements[i] - mMeasurements[i - 1]);
    }
    else
    {
      ++mDefectiveMeasurement;
    }
  }
  for (unsigned int i = 0; i < mStepSize.size(); ++i)
  {
    //   std::cout << mStepSize[i] << std::endl;
  }
}

void calculate::Calculatedata::calculateAverage()
{

  double lSum =
      std::accumulate(std::begin(mStepSize), std::end(mStepSize), 0.0);

  mMean = (lSum / static_cast<double>(mStepSize.size()));
}

void calculate::Calculatedata::calculateDeviation()
{

  double lSum = 0.0;
  std::for_each(std::begin(mStepSize), std::end(mStepSize),
                [&](const double aDistance) {
                  lSum += (aDistance - mMean) * (aDistance - mMean);
                });

  mDeviation = sqrt(lSum / (static_cast<double>(mStepSize.size() - 1)));
  // std::cout << "mean:" << mMean << "dev:" << mDeviation << std::endl;
}

void calculate::Calculatedata::processData(unsigned int aLowerBound,
                                           unsigned int aUpperBound)
{
  this->fillVector(lidar::cDataset1);
  this->calculateStepSize(aLowerBound, aUpperBound);
  this->calculateAverage();
  this->calculateDeviation();
}