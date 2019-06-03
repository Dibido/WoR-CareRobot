#include "sim_robot/StopData.hpp"
#include "sim_robot/RobotControllerPluginConst.hpp"
#include <iostream>
namespace stop_data
{

  stop_data::StopData::StopData(jointStop_t aStop) : mStop_(aStop)
  {
  }

  jointStop_t& stop_data::StopData::isStopped()
  {
    if (mStop_ != true || mStop_ != false)
    {
      throw std::invalid_argument("cStop_ must be either true or false " +
                                  std::to_string(mStop_));
    }
    return mStop_;
  }

} // namespace stop_data
