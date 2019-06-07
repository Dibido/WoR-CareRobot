#include "lidar_application/LidarData.hpp"
#include <math.h>

namespace lidar_application
{
  LidarData::LidarData()
  {
  }

  LidarData::LidarData(const std::map<double, double> aMeasurements)
      : mMeasurements(aMeasurements)
  {
    if ((aMeasurements.size() > 0) &&
        ((aMeasurements.begin()->first < 0.0) ||
         (aMeasurements.end()--->first > (2 * M_PI))))
    {
      throw std::range_error("Angle isn't a value in range [0.0 -> 2*PI]");
    }
  }

  void LidarData::reset()
  {
    mMeasurements.clear();
  }

  void LidarData::addLidarData(const std::map<double, double>& aMeasurements)
  {
    for (auto lIterator = aMeasurements.begin();
         lIterator != aMeasurements.end(); ++lIterator)
    {
      if ((lIterator->first < 0.0) || (lIterator->first > 2 * M_PI))
      {
        throw std::range_error("Angle isn't a value in range [0.0 -> 2*PI]");
      }

      mMeasurements.insert(*lIterator);
    }
  }

  void LidarData::addLidarData(const std::vector<double>& aAngles,
                               const std::vector<double>& aDistances_m)
  {
    if (aAngles.size() != aDistances_m.size())
    {
      throw std::logic_error("Sizes of given angles and distances differ");
    }

    for (size_t lIndex = 0; lIndex < aAngles.size(); ++lIndex)
    {
      mMeasurements.insert(std::pair<double, double>(aAngles.at(lIndex),
                                                     aDistances_m.at(lIndex)));
    }
  }

  void LidarData::addLidarData(double aAngle, double aDistance_m)
  {
    if ((aAngle < 0.0) || (aAngle > 2 * M_PI))
    {
      throw std::range_error("Angle isn't a value in range [0.0 -> 2*PI]");
    }

    mMeasurements.insert(std::pair<double, double>(aAngle, aDistance_m));
  }
} // namespace lidar_application