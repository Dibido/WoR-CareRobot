#ifndef KINEMATICS_ROBOTCONFIGURATION_HPP
#define KINEMATICS_ROBOTCONFIGURATION_HPP
#include "kinematics/Configuration.hpp"
#include "kinematics/KinematicsDefines.hpp"
#include "kinematics/Link.hpp"

/**
 * @brief Define PARTIAL_RANDOMISE to only randomise joints that
 * do not confirm to constraint limits in the configuration
 *
 * If PARTIAL_RANDOMISE is not defined, all joints will be randomised
 *
 */
#define PARTIAL_RANDOMISE

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
     * @brief Amount of links in the robot configuration
     *
     */
    const std::size_t size = cRobotConfigurationJoints;

    /**
     * @brief Retrieves a Link from the robotconfiguration
     *
     * @param aIndex
     * @return const Link&
     * @throws invalid_argument When aIndex is larger than size
     */
    const Link& operator[](std::size_t aIndex) const;

    /**
     * @brief Check if a configuration contains only valid values according to
     * mRobotConfiguration
     * @param aConfiguration
     * @return bool
     */
    bool isValidConfiguration(const Configuration& aConfiguration) const;

    /**
     * @brief Generate a random and valid configuration based on
     * mRobotConfiguration
     *
     * @param aConfiguration
     */
    void randomiseConfiguration(Configuration& aConfiguration) const;

      private:
    std::array<Link, cRobotConfigurationJoints> mRobotConfiguration;
  };
} // namespace kinematics

#endif // KINEMATICS_ROBOTCONFIGURATION_HPP
