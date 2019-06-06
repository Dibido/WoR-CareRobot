#include "lidar_application/LidarData.hpp"

namespace lidar_application
{
  LidarData::LidarData()
  {
  }

  LidarData::LidarData(const std::map<double, double> aMeasurements)
      : mMeasurements(aMeasurements)
  {
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
      mMeasurements.insert(
          std::pair<double, double>(aAngles.at(i), aDistances_m.at(i)));
    }
  }

  void LidarData::addLidarData(double aAngle, double aDistance_m)
  {
    mMeasurements.insert(std::pair<double, double>(aAngle, aDistance_m));
  }
} // namespace lidar_application