#include "kinematics/ConfigurationProvider.hpp"
#include "kinematics/KinematicsDefines.hpp"

namespace kinematics
{
  ConfigurationProvider::ConfigurationProvider() : denavitHartenberg()
  {
  }

  Configuration ConfigurationProvider::inverseKinematics(
      const EndEffector& aGoalEndEffector,
      const Configuration& aCurrentConfiguration)
  {
    Matrix<double, 6, 1> lEndEffector = {
      aGoalEndEffector.cX_m,       aGoalEndEffector.cY_m,
      aGoalEndEffector.cZ_m,       aGoalEndEffector.cYaw_rad,
      aGoalEndEffector.cPitch_rad, aGoalEndEffector.cRoll_rad
    };

    Configuration lConfiguration = aCurrentConfiguration;
    lConfiguration.setResult(false);
    prepareConfiguration(lConfiguration);

    lConfiguration =
        denavitHartenberg.inverseKinematics(lEndEffector, lConfiguration);

    prepareConfiguration(lConfiguration);

    return lConfiguration;
  }

  void
      ConfigurationProvider::prepareConfiguration(Configuration& aConfiguration)
  {
    for (std::size_t i = 0; i < cInvertedJoints.size(); ++i)
    {
      aConfiguration.setTheta(cInvertedJoints[i],
                              aConfiguration[cInvertedJoints[i]] * -1);
    }
  }

} // namespace kinematics
