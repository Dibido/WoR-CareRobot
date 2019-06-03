#ifndef ROBOT_CONTROLLER_CONTROL_HPP
#define ROBOT_CONTROLLER_CONTROL_HPP

#include "robotcontroller_msgs/Control.h"

#include <vector>

namespace robot_controller_control
{

  /**
   * @brief the class of the interface IRobotControlPlugin
   * @pre A control message is published on the /robot_control topic.
   * @post The control message is parsed and the joints are in correct position.
   *
   */
  class IRobotControl
  {
      public:
    /**
     * @brief parse incoming messages
     * @pre incoming: custom message with angles and speedfactor
     * @post outgoing: command with an angle, speed, movement type
     * @param aMsg The control message that we are going to parse
     */

    virtual void
        parseControlCallback(const robotcontroller_msgs::ControlPtr& aMsg) = 0;
  };
} // namespace robot_controller_control

#endif // ROBOT_CONTROLLER_CONTROL_HPP