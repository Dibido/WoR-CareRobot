#ifndef IGOAL_PROVIDER_HPP
#define IGOAL_PROVIDER_HPP

#include "environment_controller/Position.hpp"
#include "ros/ros.h"

namespace environment_controller
{

  /**
   * @brief
   *
   * @pre The desired x,y,z coordinates of where the cup needs to be placed must
   * be known
   * @post The cup will be brought (and put down) to the given coordinates
   */
  class IGoalProvider
  {

      public:
    /**
     * @brief
     *
     * @param aPosition Data object which consists of an x,y and z
     */
    virtual void selectGoalPosition(const Position& aPosition) = 0;
  };

} // namespace environment_controller

#endif // IGOAL_PROVIDER_HPP