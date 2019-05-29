#ifndef I_ROBOT_CONTROLLER_STOP_HPP
#define I_ROBOT_CONTROLLER_STOP_HPP

#include "robotcontroller_msgs/Stop.h"

#include <vector>

namespace i_robot_controller_stop
{

  /**
   * @brief the class of the interface IRobotControlPlugin
   * @pre A stop message is published on the /robot_stop topic.
   * @post The stop message is parsed and the robot is stopped.
   *
   */
  class IRobotStop
  {
      public:
    /**
     * @brief virtual interface
     *
     * @param aMsg The stop message that we are going to parse
     */
    virtual void
        parseStopCallback(const robotcontroller_msgs::StopPtr& aMsg) = 0;
  };
} // namespace i_robot_controller_stop

#endif // I_ROBOTCONTROLLER_STOP_HPP