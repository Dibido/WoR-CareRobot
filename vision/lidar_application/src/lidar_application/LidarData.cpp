#include "../include/lidar_application/LidarData.hpp"

namespace lidar_application
{
LidarData::LidarData()
{
}

LidarData::LidarData(std::vector<double>& aAngles,
                     std::vector<double>& aDistances_m)
{
  if (!(aAngles.size() == aDistances_m.size()))
  {
    throw std::logic_error("mAngles size wasn't equal to mDistances_m size");
  }

  mAngles = aAngles;
  mDistances_m = aDistances_m;
}

void LidarData::reset()
{
  mAngles.clear();
  mDistances_m.clear();
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
    mAngles.push_back(aAngles.at(i));
    mDistances_m.push_back(aDistances_m.at(i));
  }
}

void LidarData::addLidarData(double aAngle, double aDistance_m)
{
  mAngles.push_back(aAngle);
  mDistances_m.push_back(aDistance_m);
}
}