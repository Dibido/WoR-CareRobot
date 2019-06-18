#ifndef IEMERGENCYSTOP_PROVIDER_HPP
#define IEMERGENCYSTOP_PROVIDER_HPP

#include "environment_controller/Position.hpp"
#include "ros/ros.h"

namespace environment_controller
{

  /**
   * @brief Interface class which can be used to indicate if the robot needs to
   * stop
   *
   * @pre The robotarm is active
   * be known
   * @post The robot arm is stopped.
   */
  class IEmergencyStopProvider
  {

      public:
    /**
     * @brief Pure virtual function for passing the emergency stop
     *
     * @param bool aStop true or false to stop the robotarm
     */
    virtual void selectEmergencyStop(const bool aStop) = 0;
  };

} // namespace environment_controller

#endif // IEMERGENCYSTOP_PROVIDER_HPP