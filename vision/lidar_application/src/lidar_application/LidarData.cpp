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
    if ((aMeasurements.size > 0) &&
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

  void LidarData::addLidarData(std::vector<double>& aAngles,
                               std::vector<double>& aDistances_m)
  {
    if (!(aAngles.size() == aDistances_m.size()))
    {
      throw std::logic_error("mAngles size wasn't equal to mDistances_m size");
    }

    for (size_t i = 0; i < aAngles.size(); ++i)
    {
      if ((aAngles.at(i) < 0.0) || (aAngles.at(i) > 2 * M_PI))
      {
        throw std::range_error("Angle isn't a value in range [0.0 -> 2*PI]");
      }

      mMeasurements.insert(
          std::pair<double, double>(aAngles.at(i), aDistances_m.at(i)));
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