#include "sim_robot/ControlData.hpp"
#include "sim_robot/RobotControllerPluginConst.hpp"

namespace control_data
{

  control_data::CommandData::CommandData(std::vector<jointRad_t> aTheta,
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

      if (t > robotcontrollerplugin::cMaxRad ||
          t < robotcontrollerplugin::cMinRad)
      {
        throw std::invalid_argument(
            "mTheta_ must be between: " +
            std::to_string(robotcontrollerplugin::cMinRad) + " and " +
            std::to_string(robotcontrollerplugin::cMaxRad) + std::to_string(t));
      }
    }
  }

  std::vector<jointRad_t>& control_data::CommandData::mTheta()
  {
    for (const auto& t : cTheta_)
    {

      if (t > robotcontrollerplugin::cMaxRad ||
          t < robotcontrollerplugin::cMinRad)
      {
        throw std::invalid_argument(
            "mTheta_ must be between: " +
            std::to_string(robotcontrollerplugin::cMinRad) + " and " +
            std::to_string(robotcontrollerplugin::cMaxRad) + std::to_string(t));
      }
    }
    return cTheta_;
  }
  jointVel_t& control_data::CommandData::mSpeedFactor()
  {
    if (cSpeedFactor_ < 0 ||
        cSpeedFactor_ > robotcontrollerplugin::cMaxSpeedfactor)
    {
      throw std::invalid_argument(
          "cSpeedFactor_ must be between 0.0 and " +
          std::to_string(robotcontrollerplugin::cMaxSpeedfactor) + ": " +
          std::to_string(cSpeedFactor_));
    }
    return cSpeedFactor_;
  }

} // namespace data
