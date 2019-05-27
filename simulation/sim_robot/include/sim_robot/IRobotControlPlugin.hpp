#ifndef I_ROBOT_CONTROLLER_PLUGIN_HPP
#define I_ROBOT_CONTROLLER_PLUGIN_HPP

#include "sim_robot/Command.hpp"
#include "sim_robot/CommandData.hpp"

#include <vector>

namespace i_robot_controller_plugin
{

  /**
   * @brief the class of the interface IRobotControlPlugin
   * @pre a sensor detected an potential obstacle
   * @post the obstacle will be checked if it is in range of the robotarm, if
   * this is the case the robotarm will stop
   * @see Object.hpp for correct values
   */
  class IRobotControlPlugin
  {
      public:
    /**
     * @brief Construct a new IRobotControlPlugin object
     *
     */
    IRobotControlPlugin(){};

    /**
     * @brief Destroy the IRobotControlPlugin object
     *
     */
    virtual ~IRobotControlPlugin() = default;

    /**
     * @brief virtual interface
     *
     * @param aCommand The aCommand that we are going to parse
     */

    virtual void parseCallback(const data::CommandData& aCommand) = 0;

    void test();
  };
} // namespace i_robot_controller_plugin

#endif // I_ROBOTCONTROLLER_PLUGIN_HPP