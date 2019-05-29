#ifndef I_ROBOT_CONTROLLER_CONTROL_HPP
#define I_ROBOT_CONTROLLER_CONTROL_HPP

#include "robotcontroller_msgs/Control.h"

#include <vector>

namespace i_robot_controller_control
{

  /**
   * @brief the class of the interface IRobotControlPlugin
   *@pre A control message is published on the /robot_control topic.
   * @post The control message is parsed and the joints are in correct position.
   *
   */
  class IRobotControl
  {
      public:
    /**
     * @brief virtual interface
     *
     * @param aMsg The control message that we are going to parse
     */

    virtual void
        parseControlCallback(const robotcontroller_msgs::ControlPtr& aMsg) = 0;
  };
} // namespace i_robot_controller_control

#endif // I_ROBOT_CONTROLLER_CONTROL_HPP