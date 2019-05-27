#ifndef I_ROBOT_CONTROLLER_PLUGIN_HPP
#define I_ROBOT_CONTROLLER_PLUGIN_HPP

#include "sim_robot/Command.hpp"
#include "sim_robot/RobotControllerPlugin.hpp"
#include <vector>

namespace robot_controller_plugin
{

  /**
   * @brief the class of the interface IRobotControllerPlugin
   * @pre a sensor detected an potential obstacle
   * @post the obstacle will be checked if it is in range of the robotarm, if
   * this is the case the robotarm will stop
   * @see Object.hpp for correct values
   */
  class IRobotControllerPlugin
  {
      public:
    /**
     * @brief Construct a new IRobotControllerPlugin object
     *
     */
    IRobotControllerPlugin(){};

    /**
     * @brief Destroy the IRobotControllerPlugin object
     *
     */
    virtual ~IRobotControllerPlugin() = default;

    /**
     * @brief virtual interface
     *
     * @param aObstacles The obstacles that are found
     */

    virtual void parseCommands(const commands::Command& aCommands) = 0;

    void test();
    // virtual void parseObstacles(const Obstacles& aObstacles) = 0;
  };
} // namespace robot_controller_plugin

#endif // I_ROBOTCONTROLLER_PLUGIN_HPP