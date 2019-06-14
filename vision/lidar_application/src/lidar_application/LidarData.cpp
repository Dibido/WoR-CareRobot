#include "lidar_application/LidarData.hpp"

namespace lidar_application
{
  LidarData::LidarData()
  {
  }

  LidarData::LidarData(const std::map<double, double>& aMeasurements)
      : mMeasurements(aMeasurements)
  {
    // Checking if any of the measurements was out of the full-circle range
    if ((aMeasurements.size() > 0) &&
        ((aMeasurements.begin()->first < 0.0) ||
         ((aMeasurements.end()--)->first > (2 * M_PI))))
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
      rangeCheck(lIterator->first);

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
      rangeCheck(aAngles.at(lIndex));
      mMeasurements.insert(std::pair<double, double>(aAngles.at(lIndex),
                                                     aDistances_m.at(lIndex)));
    }
  }

  void LidarData::addLidarData(double aAngle, double aDistance_m)
  {
    rangeCheck(aAngle);

    mMeasurements.insert(std::pair<double, double>(aAngle, aDistance_m));
  }

  void LidarData::rangeCheck(const double& aValue,
                             const double& aMin,
                             const double& aMax) const
  {
    if ((aValue < aMin) || (aValue > aMax))
    {
      throw std::range_error(
          "Value (" + std::to_string(aValue) + ") isn't a value in range [" +
          std::to_string(aMin) + " -> " + std::to_string(aMax) + "]");
    }
  }
} // namespace lidar_application