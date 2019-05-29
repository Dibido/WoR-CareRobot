#ifndef I_ROBOT_CONTROLLER_CONTROL_HPP
#define I_ROBOT_CONTROLLER_CONTROL_HPP

#include "robotcontroller_msgs/Control.h"

#include <vector>

namespace i_robot_controller_control
{

  /**
   * @brief the class of the interface IRobotControlPlugin
   *@pre A control message is published on the /robot_control topic.
   * @post A CommandData struct is made.
   *
   */
  class IRobotControl
  {
      public:
    /**
     * @brief Construct a new IRobotControlPlugin object
     *
     */
    IRobotControl(){};

    /**
     * @brief Destroy the IRobotControlPlugin object
     *
     */
    virtual ~IRobotControl() = default;

    /**
     * @brief virtual interface
     *
     * @param aMsg The control message that we are going to parse
     */

    virtual void
        parseControlCallback(const robotcontroller_msgs::ControlPtr& aMsg) = 0;
  };
} // namespace i_robot_controller_stop

#endif // I_ROBOT_CONTROLLER_CONTROL_HPP