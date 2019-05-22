#ifndef KINEMATICS_ICONFIGURATIONPROVIDER_HPP
#define KINEMATICS_ICONFIGURATIONPROVIDER_HPP

#include "kinematics/Configuration.hpp"
#include "kinematics/EndEffector.hpp"

namespace kinematics
{
  /**
   * @brief Interface that makes it possible to find a given configuration
   * needed to reach a given endpoint from a given starting configuration
   *
   */
  class IConfigurationProvider
  {
      public:
    /**
     * @brief Uses the inverseKinematics to find a Configuration that is equal
     * to the position and rotation described in EndEffector
     *
     * @pre aGoalPosition uses the Tait-Bryan convention to describes the angles
     * yaw, pitch and roll
     * @post result in Configuration has been set to the result of the
     * inverseKinematics algorithm
     * @post If result is true, Configuration gives a configuration needed to
     * reach the EndEffector
     * @param aGoalPosition
     * @param aCurrentConfiguration
     * @return Configuration
     */
    virtual Configuration
        inverseKinematics(const EndEffector& aGoalPosition,
                          const Configuration& aCurrentConfiguration) = 0;
  };
} // namespace kinematics

#endif // KINEMATICS_ICONFIGURATIONPROVIDER_HPP
