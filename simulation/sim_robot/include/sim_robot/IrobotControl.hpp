ifndef I_OBSTACLE_HPP
#define I_OBSTACLE_HPP

#include "Object.hpp"
#include <vector>

    namespace environment_controller
{
  typedef Object Obstacle;
  typedef std::vector<Obstacle> Obstacles;
  /**
   * @brief the class of the interface robotControl
   * @pre a sensor detected an potential obstacle
   * @post the obstacle will be checked if it is in range of the robotarm, if
   * this is the case the robotarm will stop
   * @see Object.hpp for correct values
   */
  class IrobotControl
  {
      public:
    /**
     * @brief Construct a new IrobotControl object
     *
     */
    IrobotControl(){};

    /**
     * @brief Destroy the IrobotControl object
     *
     */
    virtual ~IrobotControl() = default;

    /**
     * @brief virtual interface
     *
     * @param aObstacles The obstacles that are found
     */
    // virtual void parseObstacles(const Obstacles& aObstacles) = 0;
  };
} // namespace environment_controller

#endif // I_OBSTACLE_HPP