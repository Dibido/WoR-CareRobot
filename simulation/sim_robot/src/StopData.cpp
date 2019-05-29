#include "sim_robot/StopData.hpp"
#include "sim_robot/RobotControllerPluginConst.hpp"
#include <iostream>
namespace data
{

  data::StopData::StopData(jointStop_t aStop) : cStop_(aStop)
  {
  }

  jointStop_t& data::StopData::mBool()
  {
    if (cStop_ != true || cStop_ != false)
    {
      throw std::invalid_argument("cStop_ must be either true or false " +
                                  std::to_string(cStop_));
    }
    return cStop_;
  }

} // namespace data
