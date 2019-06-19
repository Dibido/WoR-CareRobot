#ifndef IGOAL_PROVIDER_HPP
#define IGOAL_PROVIDER_HPP

#include "environment_controller/Position.hpp"
#include "ros/ros.h"

namespace environment_controller
{

  /**
   * @brief Interface class which can be used to indicate where the cup needs to
   * be placed
   *
   * @pre The desired x,y,z coordinates of where the cup needs to be placed must
   * be known
   * @post The cup will be brought (and put down) to the given coordinates
   */
  class IGoalProvider
  {

      public:
    /**
     * @brief Pure virtual function for passing the goal position
     *
     * @param aPosition Data object which consists of an x,y and z
     */
    virtual void selectGoalPosition(const Position& aPosition,
                                    bool astaticGoal) = 0;
  };

} // namespace environment_controller

#endif // IGOAL_PROVIDER_HPP