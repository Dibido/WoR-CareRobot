#ifndef I_OBSTACLE_HPP
#define I_OBSTACLE_HPP

#include "environment_controller/Object.hpp"
#include <vector>

namespace environment_controller
{
  /**
   * @brief the class of the interface obstacle
   * @pre a sensor detected an potential obstacle
   * @post the obstacles will be send over an ROS::TOPIC after this the obstacle
   * will be checked if it is in range of the robotarm, if this is the case the
   * robotarm will stop
   * @see Object.hpp for correct values
   */
  class IObstacles
  {
      public:
    /**
     * @brief Construct a new IObstacle object
     *
     */
    IObstacles(){};

    /**
     * @brief Destroy the IObstacle object
     *
     */
    virtual ~IObstacles() = default;

    /**
     * @brief the function to implement for the interface
     *
     * @param aObstacles The obstacles that are found
     */
    virtual void passObstacles(const Obstacles& aObstacles) = 0;
  };
} // namespace environment_controller

#endif // I_OBSTACLE_HPP