#include "sim_robot/ControlData.hpp"
#include "sim_robot/RobotControllerPluginConst.hpp"

namespace control_data
{

  control_data::CommandData::CommandData(std::vector<jointRad_t> aTheta,
                                 jointVel_t aSpeedFactor)
      : mTheta_(aTheta), mSpeedFactor_(aSpeedFactor)
  {
    if (mSpeedFactor_ < 0 ||
        mSpeedFactor_ > robotcontrollerplugin::cMaxSpeedfactor)
    {
      throw std::invalid_argument(
          "mSpeedFactor_ must be between 0.0 and " +
          std::to_string(robotcontrollerplugin::cMaxSpeedfactor) + ": " +
          std::to_string(mSpeedFactor_));
    }
    for (const auto& t : mTheta_)
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

  std::vector<jointRad_t>& control_data::CommandData::getTheta()
  {
    for (const auto& t : mTheta_)
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
    return mTheta_;
  }
  jointVel_t& control_data::CommandData::getSpeedFactor()
  {
    if (mSpeedFactor_ < 0 ||
        mSpeedFactor_ > robotcontrollerplugin::cMaxSpeedfactor)
    {
      throw std::invalid_argument(
          "mSpeedFactor_ must be between 0.0 and " +
          std::to_string(robotcontrollerplugin::cMaxSpeedfactor) + ": " +
          std::to_string(mSpeedFactor_));
    }
    return mSpeedFactor_;
  }

} // namespace data
