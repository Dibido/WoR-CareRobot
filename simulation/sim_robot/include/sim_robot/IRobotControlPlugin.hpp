#ifndef I_ROBOT_CONTROLLER_PLUGIN_HPP
#define I_ROBOT_CONTROLLER_PLUGIN_HPP

#include "robotcontroller_msgs/Control.h"
#include "robotcontroller_msgs/Gripper.h"
#include "robotcontroller_msgs/Stop.h"

#include <vector>

namespace i_robot_controller_plugin
{

  /**
   * @brief the class of the interface IRobotControlPlugin
   *@pre A gripperData struct is published on the /robot_control topic.
   * @post A CommandData struct is made.
   *
   */
  class IRobotControlPlugin
  {
      public:
    /**
     * @brief Construct a new IRobotControlPlugin object
     *
     */
    IRobotControlPlugin(){};

    /**
     * @brief Destroy the IRobotControlPlugin object
     *
     */
    virtual ~IRobotControlPlugin() = default;

    /**
     * @brief virtual interface
     *
     * @param aCommand The aCommand that we are going to parse
     */

    virtual void
        parseCallback(const robotcontroller_msgs::ControlPtr& aMsg) = 0;
  };
} // namespace i_robot_controller_plugin

#endif // I_ROBOTCONTROLLER_PLUGIN_HPP