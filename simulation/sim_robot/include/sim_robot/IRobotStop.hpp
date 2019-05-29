#ifndef I_ROBOT_CONTROLLER_STOP_HPP
#define I_ROBOT_CONTROLLER_STOP_HPP

#include "robotcontroller_msgs/Stop.h"

#include <vector>

namespace i_robot_controller_stop
{

  /**
   * @brief the class of the interface IRobotStop
   *@pre A stop message is published on the /robot_stop topic.
   * @post A StopData struct is made.
   *
   */
  class IRobotStop
  {
      public:
    /**
     * @brief Construct a new IRobotStop object
     *
     */
    IRobotStop(){};

    /**
     * @brief Destroy the IRobotStop object
     *
     */
    virtual ~IRobotStop() = default;

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