#ifndef IRELEASE_TIME_PROVIDER_HPP
#define IRELEASE_TIME_PROVIDER_HPP

#include "ros/ros.h"
#include <stdint.h>

namespace environment_controller
{

  /**
   * @brief
   *
   * @pre The desired amount of seconds for when a grabbed cup will be released,
   * is given by the user through the UI.
   * @post The cup will be released after the indicated amount of time
   */
  class IReleaseTimeProvider
  {

      public:
    /**
     * @brief
     *
     * @param aReleaseTime Time duration that indicates when a grabbed cup will
     * be released
     */
    virtual void selectReleaseTime(const uint8_t aReleaseTime) = 0;
  };

} // namespace environment_controller

#endif // IRELEASE_TIME_PROVIDER_HPP