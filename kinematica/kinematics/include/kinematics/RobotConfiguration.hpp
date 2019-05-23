#ifndef KINEMATICS_ROBOTCONFIGURATION_HPP
#define KINEMATICS_ROBOTCONFIGURATION_HPP
#include "kinematics/KinematicsDefines.hpp"
#include "kinematics/Link.hpp"

namespace kinematics
{
  /**
   * @brief Describes the Denavit-Hartenberg parameters for a Robot
   *
   */
  class RobotConfiguration
  {
      public:
    /**
     * @brief Construct a new Robot Configuration object
     *  Uses the Franka Emika configuration
     * @see
     * https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
     */
    RobotConfiguration();
    ~RobotConfiguration() = default;

    /**
     * @brief Retrieves a Link from the robotconfiguration
     *
     * @param aIndex
     * @return const Link&
     * @throws invalid_argument When aIndex is larger than size
     */
    const Link& operator[](std::size_t aIndex) const;
    /**
     * @brief Amount of links in the robot configuration
     *
     */
    const std::size_t size = cRobotConfigurationJoints;

      private:
    std::array<Link, cRobotConfigurationJoints> mRobotConfiguration;
  };
} // namespace kinematics

#endif // KINEMATICS_ROBOTCONFIGURATION_HPP
