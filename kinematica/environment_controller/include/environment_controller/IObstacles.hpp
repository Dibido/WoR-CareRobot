#ifndef I_OBSTACLE_HPP
#define I_OBSTACLE_HPP

#include "Object.hpp"
#include <vector>

namespace environment_controller
{

  typedef std::vector<Object> Obstacles;
  /**
   * @brief
   * @pre an sensor detected an obstacle
   * @post the obstacle will be checked if it is in range of the robotarm, if
   * this is the case the robotarm will stop
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
     * @brief virtual interface
     *
     * @param aObstacles
     */
    virtual void parseObstacles(const Obstacles& aObstacles) = 0;
  };
} // namespace environment_controller

#endif // I_OBSTACLE_HPP