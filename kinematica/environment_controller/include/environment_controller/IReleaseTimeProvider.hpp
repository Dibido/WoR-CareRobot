#ifndef IRELEASE_TIME_PROVIDER_HPP
#define IRELEASE_TIME_PROVIDER_HPP

#include "ros/ros.h"
#include <stdint.h>

namespace environment_controller
{

  /**
   * @brief Interface class whith which publishers and subscribers can be made
   * for the /release topic
   *
   * @pre The desired amount of seconds for when a grabbed cup will be released,
   * is given by the user through the UI.
   * @post The cup will be released after the indicated amount of time
   */
  class IReleaseTimeProvider
  {

      public:
    /**
     * @brief Pure virtual function for passing the release time
     *
     * @param aReleaseTime_s Time duration that indicates when a grabbed cup
     * will be released
     */
    virtual void selectReleaseTime(const uint8_t aReleaseTime_s) = 0;
  };

} // namespace environment_controller

#endif // IRELEASE_TIME_PROVIDER_HPP