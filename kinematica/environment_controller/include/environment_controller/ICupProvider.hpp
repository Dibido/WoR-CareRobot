#ifndef ICUP_PROVIDER_HPP
#define ICUP_PROVIDER_HPP

#include "environment_controller/Cup.hpp"

namespace environment_controller
{
  /**
   * @brief the interface for the cup
   * @pre the cup is detected in the vision-component, the arrival time is
   * estimated
   * @post the ROS::TOPIC is implemented and the robot will move to the desired
   * position in the desired time.
   */
  class ICupProvider
  {
      public:
    ICupProvider(){};
    virtual ~ICupProvider() = default;

    /**
     * @brief pass the cup to the function that will handle it
     *
     * @param aCup the cup
     */
    virtual void passCup(const Cup& aCup) = 0;
  };
} // namespace environment_controller

#endif // ICUP_PROVIDER_HPP