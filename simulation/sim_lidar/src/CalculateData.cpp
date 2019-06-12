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
    mMeasurements.push_back(lNum);
  }
}

void calculate::Calculatedata::calculateStepSize()
{

  for (unsigned int i = 1; i < mMeasurements.size(); ++i)
  {
    if ((mMeasurements[i] - mMeasurements[i - 1]) > 0 &&
        (mMeasurements[i] - mMeasurements[i - 1]) < 1)
    {
      mStepSize.push_back(mMeasurements[i] - mMeasurements[i - 1]);
    }
    else
    {
      mDefectiveMeasurement++;
    }
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

  double lAccum = 0.0;
  std::for_each(std::begin(mStepSize), std::end(mStepSize),
                [&](const double aDistance) {
                  lAccum += (aDistance - mMean) * (aDistance - mMean);
                });

  mDeviation = sqrt(lAccum / (static_cast<double>(mStepSize.size() - 1)));
}

void calculate::Calculatedata::processData()
{
  this->fillVector(lidar::cDataset1);
  this->calculateStepSize();
  this->calculateAverage();
  this->calculateDeviation();
}