#ifndef KINEMATICS_ICONFIGURATIONPROVIDER_HPP_
#define KINEMATICS_ICONFIGURATIONPROVIDER_HPP_

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
     * @see
     * http://wor.wiki.icaprojecten.nl/confluence/display/EBGURG/DDD+-+Kinematics+interface
     * @pre aGoalPosition uses the Tait-Bryan convention to describes the angles
     * yaw, pitch and roll
     * @post result() in Configuration returns the solution of the
     * inverseKinematics algorithm
     * @post If result is true, Configuration gives a configuration needed to
     * reach the EndEffector
     * @param aGoalPosition
     * @param aCurrentConfiguration
     * @return Configuration If execution was succesfull contains the
     * configuration needed to reach the endeffector, if unsuccesful the
     * configuration in this object is undefined and should not be used.
     */
    virtual Configuration
        inverseKinematics(const EndEffector& aGoalPosition,
                          const Configuration& aCurrentConfiguration) = 0;

    /**
     * @brief Uses the forward kinematics function to calculate endeffector
     * position for a given configuration
     *
     * @pre Configuration is valid
     * @post EndEffector position is calculated
     *
     * @param aCurrentConfiguration Configuration is a valid configuration
     * @return EndEffector Position and rotation of the EndEffector that is the
     * result of the curent configuration
     */
    virtual EndEffector
        forwardKinematics(const Configuration& aCurrentConfiguration) = 0;
  };
} // namespace kinematics

#endif // KINEMATICS_ICONFIGURATIONPROVIDER_HPP_
