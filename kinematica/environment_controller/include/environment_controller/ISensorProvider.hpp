#ifndef ISENSOR_PROVIDER_HPP
#define ISENSOR_PROVIDER_HPP

#include "ros/ros.h"

namespace environment_controller
{
  /**
   * @brief
   *
   */
  class ISensorProvider
  {
      public:
    /**
     * @brief
     *
     * @param aSensor
     */
    virtual void provideSensor(const Sensor& aSensor) = 0;
  }
} // namespace environment_controller

#endif // ISENSOR_PROVIDER_HPP