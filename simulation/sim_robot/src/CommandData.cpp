#include "sim_robot/CommandData.hpp"
#include "sim_robot/RobotControllerPluginConst.hpp"

namespace data
{

  data::CommandData::CommandData(std::vector<jointRad_t> aTheta,
                                 jointVel_t aSpeedFactor)
      : cTheta_(aTheta), cSpeedFactor_(aSpeedFactor)
  {
    if (cSpeedFactor_ < 0 ||
        cSpeedFactor_ > robotcontrollerplugin::cMaxSpeedfactor)
    {
      throw std::invalid_argument(
          "cSpeedFactor_ must be between 0.0 and " +
          std::to_string(robotcontrollerplugin::cMaxSpeedfactor) + ": " +
          std::to_string(cSpeedFactor_));
    }
    for (const auto& t : cTheta_)
    {

      if (t < 0)
      {
        throw std::invalid_argument("mTheta_ is smaller than 0: " +
                                    std::to_string(t));
      }
    }
  }

  std::vector<jointRad_t>& data::CommandData::mTheta()
  {
    for (const auto& t : cTheta_)
    {

      if (t < 0)
      {
        throw std::invalid_argument("mTheta_ is smaller than 0: " +
                                    std::to_string(t));
      }
    }
    return cTheta_;
  }
  jointVel_t& data::CommandData::mSpeedFactor()
  {
    if (cSpeedFactor_ < 0)
    {
      throw std::invalid_argument("B is smaller than 0: " +
                                  std::to_string(cSpeedFactor_));
    }
    return cSpeedFactor_;
  }

} // namespace data
