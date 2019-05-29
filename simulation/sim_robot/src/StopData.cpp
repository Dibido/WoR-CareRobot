#include "sim_robot/StopData.hpp"
#include "sim_robot/RobotControllerPluginConst.hpp"
#include <iostream>
namespace stop_data
{

  stop_data::StopData::StopData(jointStop_t aStop) : cStop_(aStop)
  {
  }

  jointStop_t& stop_data::StopData::mBool()
  {
    if (cStop_ != true || cStop_ != false)
    {
      throw std::invalid_argument("cStop_ must be either true or false " +
                                  std::to_string(cStop_));
    }
    return cStop_;
  }

} // namespace stop_data
