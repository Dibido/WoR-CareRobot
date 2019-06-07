#include "sim_lidar/CalculateData.hpp"
#include <algorithm>
#include <math.h>

calculate::Calculatedata::Calculatedata()
    : mMeasurements(0.0), mStepSize(0, 0), mMean(0.0), mDeviation(0)
{
}

calculate::Calculatedata::Calculatedata(std::vector<double> aMeasurements)
    : mMeasurements(aMeasurements), mStepSize(0, 0), mMean(0.0), mDeviation(0)
{
}

void calculate::Calculatedata::fillVector(std::string Afile)
{

  std::ifstream ifile(Afile, std::ios::in);

  if (!ifile.is_open())
  {
    std::cerr << "There was a problem opening the input file!\n";
    exit(1);
  }

  double num = 0.0;

  while (ifile >> num)
  {
    mMeasurements.push_back(num);
  }

  for (unsigned int i = 0; i < mMeasurements.size(); ++i)
  {
    std::cout << mMeasurements[i] << std::endl;
  }
}

void calculate::Calculatedata::calculateStepSize()
{

  for (unsigned int i = 1; i < mMeasurements.size(); ++i)
  {
    mStepSize.push_back(mMeasurements[i] - mMeasurements[i - 1]);
    std::cout << "stepsize:" << mMeasurements[i] - mMeasurements[i - 1]
              << std::endl;
  }
}

void calculate::Calculatedata::calculateAverage()
{

  double lSum =
      std::accumulate(std::begin(mStepSize), std::end(mStepSize), 0.0);

  mMean = (lSum / mStepSize.size());
  std::cout << "mean:" << mMean << std::endl;
}

void calculate::Calculatedata::calculateDeviation()
{

  double lAccum = 0.0;
  std::for_each(std::begin(mStepSize), std::end(mStepSize),
                [&](const double aDistance) {
                  lAccum += (aDistance - mMean) * (aDistance - mMean);
                });

  mDeviation = sqrt(lAccum / (mStepSize.size() - 1));
  std::cout << "deviation:" << mDeviation << std::endl;
}

void calculate::Calculatedata::processData(std::string aFile)
{
  this->fillVector(aFile);
  this->calculateStepSize();
  this->calculateAverage();
  this->calculateDeviation();
}